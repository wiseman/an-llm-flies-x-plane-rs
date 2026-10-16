# /// script
# requires-python = ">=3.11"
# dependencies = [
#     "markdown-it-py[plugins]",
#     "pygments",
#     "pyyaml",
# ]
# ///
"""Render DRAFT.md (+ an appendix built from instructions.yaml) to DRAFT.html.

Self-contained output: inline CSS, no external assets.
"""

import html
import re
from pathlib import Path

import yaml
from markdown_it import MarkdownIt
from mdit_py_plugins.anchors import anchors_plugin
from pygments import highlight
from pygments.formatters import HtmlFormatter
from pygments.lexers import get_lexer_by_name

ROOT = Path(__file__).parent


def highlight_code(code: str, lang: str, _attrs: str) -> str:
    try:
        lexer = get_lexer_by_name(lang or "text")
    except Exception:
        lexer = get_lexer_by_name("text")
    return highlight(code, lexer, HtmlFormatter(nowrap=True))


def appendix() -> str:
    items = yaml.safe_load((ROOT / "instructions.yaml").read_text())
    rows = []
    for it in items:
        raw = it.get("raw")
        raw_html = f'<div class="raw">{html.escape(raw)}</div>' if raw else ""
        rows.append(
            f'<tr id="item-{it["id"]}" data-family="{it["family"]}">'
            f'<td class="id">{it["id"]}</td>'
            f'<td class="src">{html.escape(it["source"])}</td>'
            f'<td>{html.escape(it["text"])}{raw_html}</td></tr>'
        )
    families = sorted({it["family"] for it in items})
    buttons = "".join(f'<button data-f="{f}">{f.replace("_", " ")}</button>' for f in families)
    return f"""
<h2 id="appendix-the-curated-corpus">Appendix: the curated corpus</h2>
<p>All {len(items)} items from <code>instructions.yaml</code>. ATC items show the cleaned reading and,
beneath it, the raw Whisper line it came from.</p>
<div class="filters"><button data-f="" class="on">all</button>{buttons}</div>
<div class="table-wrap"><table class="corpus"><thead><tr><th>ID</th><th>Source</th><th>Instruction</th></tr></thead>
<tbody>{''.join(rows)}</tbody></table></div>
"""


def main() -> None:
    md = (
        MarkdownIt("commonmark", {"highlight": highlight_code, "typographer": True})
        .enable(["table", "replacements", "smartquotes"])
        .use(anchors_plugin, min_level=2, max_level=3, permalink=False)
    )
    src = (ROOT / "DRAFT.md").read_text()
    title_match = re.match(r"# (.+)\n", src)
    title = title_match.group(1) if title_match else "Draft"
    body = md.render(src[title_match.end():] if title_match else src)

    # Wrap tables so they scroll horizontally on narrow screens, and link
    # instruction IDs in the first column to their corpus rows.
    body = body.replace("<table>", '<div class="table-wrap"><table>').replace("</table>", "</table></div>")
    body = re.sub(r"<td>([A-P]\d{1,2})</td>", r'<td class="id"><a href="#item-\1">\1</a></td>', body)
    body += appendix()

    toc = "".join(
        f'<a class="l{lvl}" href="#{aid}">{re.sub("<[^>]+>", "", txt)}</a>'
        for lvl, aid, txt in re.findall(r'<h([23]) id="([^"]+)">(.*?)</h\1>', body)
    )
    pyg_light = HtmlFormatter(style="friendly").get_style_defs(".hl")
    pyg_dark = HtmlFormatter(style="monokai").get_style_defs(".hl")

    out = TEMPLATE.format(title=html.escape(title), toc=toc, body=body,
                          pyg_light=pyg_light, pyg_dark=scope_dark(pyg_dark))
    # Pygments emits bare spans; give code blocks the .hl scope.
    out = out.replace("<pre><code", '<pre class="hl"><code')
    (ROOT / "DRAFT.html").write_text(out)
    print(ROOT / "DRAFT.html")


def scope_dark(css: str) -> str:
    # Dark code colours follow the page theme: the OS setting unless the
    # host forces data-theme="light", or always under data-theme="dark".
    auto = css.replace(".hl", ':root:not([data-theme="light"]) .hl')
    forced = css.replace(".hl", ':root[data-theme="dark"] .hl')
    rules = "\n".join(f"  {line}" for line in auto.splitlines())
    return "@media (prefers-color-scheme: dark) {\n" + rules + "\n}\n" + forced + "\n"


TEMPLATE = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Pilot Vocabulary API</title>
<style>
:root {{
  --bg: #fbfaf7; --panel: #f2f0ea; --text: #1d1f22; --muted: #62666d;
  --rule: #dedad0; --accent: #1f5f8b; --accent-soft: #e3eef6; --code-bg: #f4f2ec;
  --mono: ui-monospace, "SF Mono", Menlo, Consolas, monospace;
  --sans: -apple-system, BlinkMacSystemFont, "Segoe UI", "Helvetica Neue", Arial, sans-serif;
}}
@media (prefers-color-scheme: dark) {{
  :root:not([data-theme="light"]) {{
    --bg: #15171a; --panel: #1c1f23; --text: #e6e4df; --muted: #9a9ea5;
    --rule: #2f333a; --accent: #7fb6dd; --accent-soft: #1d2c38; --code-bg: #1e2126;
  }}
}}
:root[data-theme="dark"] {{
  --bg: #15171a; --panel: #1c1f23; --text: #e6e4df; --muted: #9a9ea5;
  --rule: #2f333a; --accent: #7fb6dd; --accent-soft: #1d2c38; --code-bg: #1e2126;
}}
* {{ box-sizing: border-box; }}
html {{ scroll-padding-top: 16px; }}
body {{ margin: 0; background: var(--bg); color: var(--text); font: 16px/1.6 var(--sans); }}
.layout {{ display: grid; grid-template-columns: 250px minmax(0, 1fr); max-width: 1320px; margin: 0 auto; }}
nav {{ position: sticky; top: 0; align-self: start; height: 100vh; overflow-y: auto;
       padding: 28px 18px 28px 24px; border-right: 1px solid var(--rule); font-size: 13.5px; }}
nav .brand {{ font-weight: 650; font-size: 14px; margin-bottom: 14px; letter-spacing: .01em; }}
nav a {{ display: block; color: var(--muted); text-decoration: none; padding: 3px 0; line-height: 1.35; }}
nav a:hover {{ color: var(--accent); }}
nav a.l3 {{ padding-left: 12px; font-size: 13px; }}
main {{ padding: 36px 48px 80px; min-width: 0; }}
header h1 {{ font-size: 30px; line-height: 1.2; margin: 0 0 6px; letter-spacing: -.01em; }}
header .kicker {{ text-transform: uppercase; letter-spacing: .12em; font-size: 12px; color: var(--accent); font-weight: 600; }}
h2 {{ font-size: 23px; margin: 52px 0 12px; padding-top: 12px; border-top: 1px solid var(--rule); }}
h3 {{ font-size: 18px; margin: 34px 0 8px; }}
p, li {{ max-width: 76ch; }}
a {{ color: var(--accent); }}
hr {{ display: none; }}
code {{ font: 0.86em/1.5 var(--mono); background: var(--code-bg); padding: 1px 5px; border-radius: 4px; }}
pre {{ background: var(--code-bg); border: 1px solid var(--rule); border-radius: 8px; padding: 14px 16px;
       overflow-x: auto; font: 13px/1.55 var(--mono); }}
pre code {{ background: none; padding: 0; font: inherit; }}
.table-wrap {{ overflow-x: auto; margin: 14px 0 18px; border: 1px solid var(--rule); border-radius: 8px; }}
table {{ border-collapse: collapse; width: 100%; font-size: 14px; }}
th, td {{ text-align: left; vertical-align: top; padding: 8px 12px; border-bottom: 1px solid var(--rule); }}
th {{ background: var(--panel); font-weight: 600; font-size: 13px; white-space: nowrap; }}
tr:last-child td {{ border-bottom: none; }}
td code {{ font-size: 12.5px; white-space: normal; word-break: break-word; }}
td[style*="right"] {{ font-variant-numeric: tabular-nums; }}
td.id {{ font: 600 12.5px var(--mono); white-space: nowrap; }}
td.id a {{ text-decoration: none; }}
td.src {{ white-space: nowrap; color: var(--muted); font-size: 13px; }}
.raw {{ margin-top: 4px; color: var(--muted); font: 12px/1.45 var(--mono); }}
.raw::before {{ content: "raw: "; opacity: .7; }}
tr:target td {{ background: var(--accent-soft); }}
.filters {{ display: flex; flex-wrap: wrap; gap: 6px; margin: 10px 0; }}
.filters button {{ font: 12.5px var(--sans); border: 1px solid var(--rule); background: var(--panel);
                   color: var(--text); padding: 4px 10px; border-radius: 999px; cursor: pointer; }}
.filters button.on {{ background: var(--accent); color: var(--bg); border-color: var(--accent); }}
{pyg_light}
{pyg_dark}
.hl {{ background: var(--code-bg); }}
@media (max-width: 900px) {{
  .layout {{ grid-template-columns: 1fr; }}
  nav {{ display: none; }}
  main {{ padding: 24px 16px 60px; }}
  header h1 {{ font-size: 24px; }}
}}
</style>
</head>
<body>
<div class="layout">
<nav><div class="brand">Pilot vocabulary API</div>{toc}</nav>
<main>
<header><div class="kicker">Design draft · an-llm-flies-x-plane</div><h1>{title}</h1></header>
{body}
</main>
</div>
<script>
document.querySelectorAll('.filters button').forEach(b => b.addEventListener('click', () => {{
  document.querySelectorAll('.filters button').forEach(x => x.classList.toggle('on', x === b));
  const f = b.dataset.f;
  document.querySelectorAll('table.corpus tbody tr').forEach(r => {{
    r.style.display = !f || r.dataset.family === f ? '' : 'none';
  }});
}}));
</script>
</body>
</html>
"""

if __name__ == "__main__":
    main()
