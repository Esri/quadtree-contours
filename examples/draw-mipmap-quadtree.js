// © 2019 3D Robotics. License: Apache-2.0
//
// Generate a series of frames showing the internal quadtree while sweeping
// through contour levels.
//
// Usage: 
// node examples/draw-mipmap-quadtree.js DEM.tif
// ffmpeg -i out_%4d.png -vf "palettegen=max_colors=16" -y pal.png
// ffmpeg -framerate 10 -i out_%4d.png -i pal.png -lavfi "paletteuse=dither=none" -y out.gif
//

const fs = require('fs');
const util = require('util');
const readFile = util.promisify(fs.readFile);
const writeFile = util.promisify(fs.writeFile);
const Canvas = require('canvas');
const Geotiff = require('geotiff');
const ContourMipmap = require('..');

async function run() {
    const buffer = await readFile(process.argv[2]);
    const arrayBuffer = buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.byteLength);
    const demTiff = await Geotiff.fromArrayBuffer(arrayBuffer);
    const image = await demTiff.getImage();

    const imageData = await image.readRasters();

    // Replace the NoDataValue in the array with NaN
    function removeNoDataValue(raster) {
        const buf = new Float32Array(raster);
        for (let i = 0; i < buf.length; i++) {
            if (buf[i] < -1000) { buf[i] = NaN; }
        }
        return buf;
    }

    const buf = removeNoDataValue(imageData[0]);

    const mipmap = new ContourMipmap(buf, imageData.width, imageData.height);

    let size = 800;
    let canvas = new Canvas(size, size);
    let ctx = canvas.getContext('2d');

    function addLine(l, x1, y1, x2, y2) {
        const scale = size / Math.pow(2, l);
        ctx.strokeStyle = 'black';
        ctx.strokeWidth = 1;
        ctx.beginPath();
        ctx.moveTo(scale * x1, scale * y1);
        ctx.lineTo(scale * x2, scale * y2);
        ctx.stroke();
    }

    function addNode(l, x, y, e) {
        const scale = size / Math.pow(2, l);
        ctx.fillStyle = ['rgba(0, 0, 0, 0)', 'rgba(0, 255, 0, 0.1)', 'rgba(255, 0, 0, 0.1)'][e];
        ctx.strokeStyle = 'rgba(0, 0, 0, 0.05)';
        ctx.strokeWidth = 2;
        ctx.fillRect(scale * x, scale * y, scale, scale);
        ctx.strokeRect(scale * x, scale * y, scale, scale);
    }

    let minElev = Math.floor(mipmap.levels[0].min[0]);
    let maxElev = Math.ceil(mipmap.levels[0].max[0]);

    let frame = 0;
    for (let elev = minElev; elev < maxElev; elev += 0.15) {
        console.log(frame, minElev, elev, maxElev);
        ctx.fillStyle = 'white';
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        mipmap.evaluateContour(elev, {}, addLine, addNode);
        await writeFile(`out_${('0000' + frame).slice(-4)}.png`, canvas.toBuffer());
        frame++
    }  
}

run()

