import sharp from 'sharp';
const [,, file, out] = process.argv;
await sharp(file, { density: 144 }).png().toFile(out);
console.log('ok', out);
