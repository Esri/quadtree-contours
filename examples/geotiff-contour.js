// © 2019 3D Robotics. License: Apache-2.0
//
// Generate GeoJSON contours from a GeoTIFF DEM file
//
// usage: `node examples/geotiff-contour.js in.tiff 10 out.geo.json`
// (assumes that the input file is in WGS84 for GeoJSON; coordinates are transferred directly)
//

const fs = require('fs');
const util = require('util');
const readFile = util.promisify(fs.readFile);
const writeFile = util.promisify(fs.writeFile);
const Canvas = require('canvas');
const Geotiff = require('geotiff');
const ContourMipmap = require('..');

function removeNoDataValue(raster) {
    const buf = new Float32Array(raster);
    for (let i = 0; i < buf.length; i++) {
        if (buf[i] < -1000) { buf[i] = NaN; }
    }
    return buf;
}

async function run(inFile, interval, outFile) {
    const buffer = await readFile(inFile);
    const arrayBuffer = buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.byteLength);
    const demTiff = await Geotiff.fromArrayBuffer(arrayBuffer);
    const image = await demTiff.getImage();
    const imageData = await image.readRasters();
    const buf = removeNoDataValue(imageData[0]);
    const mipmap = new ContourMipmap(buf, imageData.width, imageData.height);

    const [originX, originY] = image.getOrigin();
    const [scaleX, scaleY] = image.getResolution();

    const features = mipmap.intervals(interval).map((level) => {
        const lines = mipmap.contour(level);

        const coordinates = lines.map(coords => 
            coords.map(([x, y]) => [originX + x * scaleX, originY + y * scaleY])
        );

        return {
            type: 'Feature',
            geometry: {
                type: 'MultiLineString',
                coordinates,
            },
            properties: {
                level: level,
            },
        };
    });

    const geojson =  {
        type: 'FeatureCollection',
        features,
    };

    await writeFile(outFile, JSON.stringify(geojson));
}

if (typeof module != 'undefined' && !module.parent) {
    run(process.argv[2], parseInt(process.argv[3], 10), process.argv[4])
}

