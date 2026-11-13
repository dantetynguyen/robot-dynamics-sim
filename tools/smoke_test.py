"""Browser smoke test: load the site in headless Chrome, open every tab, set up
and briefly run every experiment, type a syntax error into an equation, switch
theme and mode. Fails (exit code 1) on any page error, failed request or
console error. WebGL driver warnings from software rendering are ignored.

Needs: pip install playwright, a Chrome/Chromium binary, and the site served
over http (ES modules do not load from file://).

    python3 -m http.server 8080 &
    python3 tools/smoke_test.py [url] [screenshot.png]
"""
import asyncio
import json
import shutil
import sys

from playwright.async_api import async_playwright

URL = sys.argv[1] if len(sys.argv) > 1 else "http://localhost:8080/index.html"
SHOT = sys.argv[2] if len(sys.argv) > 2 else None
CHROME = shutil.which("google-chrome") or shutil.which("chromium") or shutil.which("chromium-browser")

SCRIPT = r"""(async () => {
  const wait = (ms) => new Promise((r) => setTimeout(r, ms));
  const out = {};
  const { EXPERIMENTS } = await import('/src/teach/experiments.js');
  // every tab in dynamics mode
  for (const tab of ['control', 'wrench', 'equations', 'learn', 'checks', 'robot']) { app.setMode('dynamics', tab); await wait(350); }
  // every experiment: set up through the real button path, run briefly
  for (const ex of EXPERIMENTS) {
    app.setMode(ex.mode, 'learn');
    const btn = [...document.querySelectorAll('.card')].find((c) => c.querySelector('h4')?.textContent === ex.title)?.querySelector('button');
    btn.click();
    await wait(150);
    if (ex.mode === 'dynamics') { app.setRunning(true); await wait(900); app.setRunning(false); }
    else await wait(400);
    out[ex.id] = { t: +app.sim.t.toFixed(2), fault: app.sim.fault?.message ?? null, banner: app.banner.hidden ? null : app.banner.textContent };
    for (const a of document.querySelectorAll('.card.active .row button')) if (/diag|Coriolis|Restore/.test(a.textContent)) { a.click(); await wait(200); }
  }
  // an equation syntax error must not stop the run
  app.setMode('dynamics', 'equations');
  const ta = document.querySelector('.eq textarea');
  ta.value = 'G*Vd +* ('; ta.dispatchEvent(new Event('input'));
  app.setRunning(true); await wait(600); app.setRunning(false);
  out.syntaxError = { fault: app.sim.fault?.message ?? null, status: document.querySelector('.eq .status-line').textContent.slice(0, 80) };
  app.setTheme('light'); await wait(200);
  app.setMode('kinematics'); await wait(400);
  return out;
})()
"""


async def main():
    async with async_playwright() as p:
        browser = await p.chromium.launch(
            executable_path=CHROME,
            args=["--no-sandbox", "--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"])
        page = await browser.new_page(viewport={"width": 1440, "height": 900})
        problems = []
        page.on("console", lambda m: problems.append(f"[console {m.type}] {m.text}")
                if m.type == "error" else None)
        page.on("pageerror", lambda e: problems.append(f"[pageerror] {e}"))
        page.on("requestfailed", lambda r: problems.append(f"[requestfailed] {r.url}"))
        await page.goto(URL)
        await page.wait_for_timeout(1500)
        result = await page.evaluate(SCRIPT)
        await page.wait_for_timeout(500)
        if SHOT:
            await page.screenshot(path=SHOT)
        await browser.close()
    faults = {k: v for k, v in result.items() if v.get("fault") or v.get("banner")}
    print(json.dumps(result, indent=1))
    for line in problems:
        print(line[:300])
    if problems or faults:
        print(f"FAILED: {len(problems)} browser problem(s), {len(faults)} experiment fault(s)")
        sys.exit(1)
    print("smoke test passed: no page errors, no failed requests, no faults")


asyncio.run(main())
