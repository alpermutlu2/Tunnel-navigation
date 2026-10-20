// Frame-stepped recording of a simulator page (needs `npm i playwright`; set PW_CHROMIUM to use a
// specific Chromium binary): node tools/record_frames.js <abs path to sim html> <frame dir>
// then: ffmpeg -framerate 20 -i <frame dir>/f%04d.png -c:v libx264 -pix_fmt yuv420p out.mp4
const { chromium } = require('playwright');
(async () => {
  const b = await chromium.launch(process.env.PW_CHROMIUM ? { executablePath: process.env.PW_CHROMIUM } : {});
  const p = await b.newPage({ viewport: { width: 1300, height: 860 } });
  await p.goto('file://' + process.argv[2]);
  await p.waitForTimeout(800);
  const [k0, k1, dk] = await p.evaluate(() => {
    const a = D.t.findIndex(t => t >= D.t_entry - 15), e = D.t.findIndex(t => t >= D.t_exit + 45);
    const dt = D.t[1] - D.t[0]; return [a, e > 0 ? e : D.t.length - 1, Math.max(1, Math.round(0.5 / dt))];
  });
  await p.evaluate(() => { document.getElementById('zoom').value = 0.6; });
  let n = 0;
  for (let kk = k0; kk <= k1; kk += dk) {
    await p.evaluate(x => { k = x; render(); }, kk);
    await p.screenshot({ path: `${process.argv[3]}/f${String(n++).padStart(4, '0')}.png` });
  }
  await b.close(); console.log('frames', n);
})();
