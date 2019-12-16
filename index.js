// © 2019 3D Robotics. License: Apache-2.0
/* eslint no-plusplus: "off", prefer-rest-params: "off" */

const OUTSIDE = 0;
const ABOVE = 1;
const BELOW = 2;
const WITHIN = 3;

// Build mipmap layer N from layer N+1
function mipmapReduce({ min, max, width, height, scale }) {
  const outWidth = Math.ceil(width / 2);
  const outHeight = Math.ceil(height / 2);

  function reduceWith(data, reduce) {
    const out = new Float32Array(outWidth * outHeight);
    let x;
    let y;

    for (y = 0; y < Math.floor(height / 2); y++) {
      for (x = 0; x < Math.floor(width / 2); x++) {
        out[y * outWidth + x] = reduce(
          data[(y * 2 + 0) * width + (x * 2 + 0)], data[(y * 2 + 0) * width + (x * 2 + 1)],
          data[(y * 2 + 1) * width + (x * 2 + 0)], data[(y * 2 + 1) * width + (x * 2 + 1)],
        );
      }
      if (x < outWidth) {
        out[y * outWidth + x] = reduce(
          data[(y * 2 + 0) * width + (x * 2 + 0)],
          data[(y * 2 + 1) * width + (x * 2 + 0)],
        );
      }
    }
    if (y < outHeight) {
      for (x = 0; x < Math.floor(width / 2); x++) {
        out[y * outWidth + x] = reduce(
          data[(y * 2 + 0) * width + (x * 2 + 0)], data[(y * 2 + 0) * width + (x * 2 + 1)],
        );
      }
    }
    if (x < outWidth && y < outHeight) {
      out[y * outWidth + x] = reduce(data[(y * 2 + 0) * width + (x * 2 + 0)]);
    }
    return out;
  }

  function minFinite() {
    let r = Infinity;
    for (let i = 0; i < arguments.length; i++) { // significantly faster than the code Babel generates for ...args
      const v = arguments[i];
      if (v < r) r = v;
    }
    return r;
  }

  function maxFinite() {
    let r = -Infinity;
    for (let i = 0; i < arguments.length; i++) {
      const v = arguments[i];
      if (v > r) r = v;
    }
    return r;
  }

  return {
    min: reduceWith(min, minFinite),
    max: reduceWith(max, maxFinite),
    width: outWidth,
    height: outHeight,
    scale: scale * 2,
  };
}

class ContourMipmap {
  constructor(raster, width, height) {
    // Bottom mipmap layer is just the raster: at each pixel the minimum and maximum are the same
    this.levels = [{ min: raster, max: raster, width, height, scale: 1 }];

    while (this.levels[0].width > 1 || this.levels[0].height > 1) {
      this.levels.unshift(mipmapReduce(this.levels[0]));
    }
  }

  // Get the minimum elevation in the mipmap
  min() { return this.levels[0].min[0] }

  // Get the maximum elevation in the mipmap
  max() { return this.levels[0].max[0] }

  // Get an array of contour levels for a specified contour interval
  intervals(interval) {
    if (interval <= 0) throw new Error('Contour interval must be positive');

    const min = this.min();
    const max = this.max();

    const levels = [];
    let level = Math.ceil(min / interval) * interval;
    while (level < max) {
      levels.push(level);
      level += interval;
    }

    return levels;
  }

  // Walk the quadtree for a specified contour level, and pass quadtree nodes
  // and contour line segments to the callbacks. This is used for visualizing
  // the quadtree; in most cases you should use .contour() instead, which
  // assembles the line segments into lines and rings.
  evaluateContour(contourLevel, { maxMipmapLevel }, addLine, addNode) {
    maxMipmapLevel = maxMipmapLevel || this.levels.length;

    // Test the mipmap at the given coordinate and level, returning whether the contour line is ABOVE, BELOW, or WITHIN this mipmap cell
    // If the mipmap is out of bounds or NaN at that point, OUTSIDE is returned.
    const evaluate = (l, x, y) => {
      const ml = this.levels[l];
      const i = ml.width * y + x;
      if (x >= ml.width || y >= ml.height || !Number.isFinite(ml.min[i]) || !Number.isFinite(ml.max[i])) return OUTSIDE;
      if (ml.min[i] >= contourLevel) return ABOVE;
      if (ml.max[i] < contourLevel) return BELOW;

      // If we've reached the maximum mipmap level, arbitrarily decide that cells that contain
      // the contour elevation go inside the contour line.
      if (l >= maxMipmapLevel) return ABOVE;

      return WITHIN;
    }

    // Add any vertical contour lines that exist between passed mipmap cell and
    // its horizontal neighbor to the right. If the line exists at this mipmap
    // level, it is added directly, otherwise split the edge and recurse at a
    // more detailed mipmap level.
    function edgeH(l, x, y) {
      const e1 = evaluate(l, x, y);
      const e2 = evaluate(l, x+1, y);

      if (e1 === ABOVE && e2 === BELOW) {
        addLine(l, x+1, y+1, x+1, y);
      } else if (e2 === ABOVE && e1 === BELOW) {
        // reverse order of coordinates compared to case above so that lines
        // go counterclockwise around the region above the contour value
        addLine(l, x+1, y,     x+1, y+1);
      } else if (e1 === WITHIN || e2 === WITHIN) {
        edgeH(l+1, x*2+1, y*2+0);
        edgeH(l+1, x*2+1, y*2+1);
      }
    }

    // Like edgeH, but for horizontal contour lines between the passed cell
    // and its vertical neighbor below.
    function edgeV(l, x, y) {
      const e1 = evaluate(l, x, y);
      const e2 = evaluate(l, x, y+1);

      if (e1 === ABOVE && e2 === BELOW) {
        addLine(l, x, y+1, x+1, y+1);
      } else if (e2 === ABOVE && e1 === BELOW) {
        addLine(l, x+1, y+1, x, y+1);
      } else if (e1 === WITHIN || e2 === WITHIN) {
        edgeV(l+1, x*2+0, y*2+1);
        edgeV(l+1, x*2+1, y*2+1);
      }
    }

    // Add all contour lines within a mipmap cell.
    // It recurses into the four quadrants of the cell, and also uses edgeH and
    // edgeV to add the contour lines that exist between quadrants.
    function node(l, x, y) {
      let e = evaluate(l, x, y);
      if (e === WITHIN) {
        node(l+1, x*2+0, y*2+0);
        node(l+1, x*2+1, y*2+0);
        node(l+1, x*2+0, y*2+1);
        node(l+1, x*2+1, y*2+1);
        edgeH(l+1, x*2+0, y*2+0);
        edgeV(l+1, x*2+0, y*2+0);
        edgeH(l+1, x*2+0, y*2+1);
        edgeV(l+1, x*2+1, y*2+0);
      } else {
        if (addNode) addNode(l, x, y, e);
      }
    }

    node(0, 0, 0);
  }

  // Generate a contour line for a specified value
  //
  // level: Value in the input array at which to draw a contour line
  // opts: {
  //   maxMipmapLevel: Maximum recursion depth in the mipmap. As if you rescaled
  //                   the input to a maximum dimension of 2^n pixels.
  //                   (default unlimited)
  //   minPoints: Lines and rings with fewer points are removed. (default 0)
  //   smoothKernelWidth: Width of the rectangular filter for smoothing (default 2)
  //   smoothCycles: Number of iterations of smoothing (default 2)
  // }
  //
  // Returns an array of lines, where each line is an array of [x, y] pairs.
  contour(level, { maxMipmapLevel, smoothKernelWidth = 2, smoothCycles = 2, minPoints = 0 } = {}) {
    // Use the quadtree algorithm to generate line segments that make up the contour
    const segments = [];
    this.evaluateContour(level, { maxMipmapLevel }, (l, x1, y1, x2, y2) => {
      const scale = this.levels[l].scale;

      const start = [scale * x1, scale * y1];
      const end = [scale * x2, scale * y2];

      segments.push({ start, end });
    });

    segments.sort((a, b) => key(a.start) - key(b.start));

    // .. and then join them together into rings

    // An arbitrary function to use a 2D point as a Map key
    function key(a) { return a[0] + a[1] * 65536; }

    const fragmentStart = new Map();
    const fragmentEnd = new Map();
    const rings = [];
  
    segments.forEach(({ start, end }) => {
      const a = fragmentEnd.get(key(start));
      const b = fragmentStart.get(key(end));

      if (a && b) {
        fragmentEnd.delete(key(start));
        fragmentStart.delete(key(end));

        if (a === b) {
          // Close a ring
          a.push(end);
          rings.push(a);
        } else {
          // Join two lines together
          const c = a.concat(b);
          fragmentStart.set(key(c[0]), c);
          fragmentEnd.set(key(c[c.length - 1]), c);
        }
      } else if (a) {
        // Extend an existing line from the end
        fragmentEnd.delete(key(start));
        a.push(end);
        fragmentEnd.set(key(end), a);
      } else if (b) {
        // Extend an existing line from the start
        fragmentStart.delete(key(end));
        b.unshift(start);
        fragmentStart.set(key(start), b);
      } else {
        // New line doesn't connect to any existing line
        const c = [start, end];
        fragmentStart.set(key(start), c);
        fragmentEnd.set(key(end), c);
      }

      if (fragmentStart.size !== fragmentEnd.size) {
        // TODO: is there a better assert function to use?
        // eslint-disable-next-line
        console.error(`Contour remaining fragment size mismatch ${fragmentStart.size} ${fragmentEnd.size}`);
      }
    });

    const smoothOpts = {
      kernelWidth: smoothKernelWidth,
      cycles: smoothCycles,
    };

    // Closed rings and any unclosed line strings
    return [...rings, ...fragmentStart.values()]
      .filter(l => l.length >= minPoints && l.length >= smoothKernelWidth * 2)
      .map(l => smooth(l, smoothOpts))
  }
}

// Smooth a line by repeatedly applying a rectangular filter to approximate
// a gaussian filter. It detects closed loops and preserves them.
// The line is passed as an array of `[x, y]` pairs.
//
// kernelWidth: Width of the rectangular filter kernel
// cycles: Number of times to apply the kernel
function smooth(line, { kernelWidth = 2, cycles = 2 }) {
  const isLoop = line[0][0] === line[line.length - 1][0] && line[0][1] === line[line.length - 1][1];

  let pointAt;
  if (isLoop) {
    pointAt = i => line[(i + line.length) % line.length];
  } else {
    pointAt = i => line[Math.min(Math.max(i, 0), line.length - 1)];
  }

  const sc = 1.0 / (kernelWidth * 2);

  for (let cycle = 0; cycle < cycles; cycle++) {
    let px = 0;
    let py = 0;

    if (isLoop) {
      for (let i = line.length - kernelWidth; i < line.length; i++) {
        px += line[i][0];
        py += line[i][1];
      }
    } else {
      px = line[0][0] * kernelWidth;
      py = line[0][1] * kernelWidth;
    }

    for (let i = 0; i < kernelWidth; i++) {
      px += line[i][0];
      py += line[i][1];
    }

    const res = line.slice();

    for (let i = 0; i < line.length; i++) {
      res[i] = [px * sc, py * sc];
      px += pointAt(i + kernelWidth)[0] - pointAt(i - kernelWidth)[0];
      py += pointAt(i + kernelWidth)[1] - pointAt(i - kernelWidth)[1];
    }

    if (isLoop) {
      // Keep the loop closed
      res[res.length - 1] = res[0].slice();
    }

    line = res;
  }

  return line;
}

module.exports = ContourMipmap;
