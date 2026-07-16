#!/usr/bin/env python3
"""Export USER_MANUAL.md to a print-ready PDF with Chromium."""

from __future__ import annotations

import argparse
import html
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

from PIL import Image
from pypdf import PdfReader, PdfWriter


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT / "USER_MANUAL.md"
DEFAULT_OUTPUT = ROOT / "USER_MANUAL.pdf"
SOURCE_IMAGE = ROOT / "assets" / "yummyimu-product.png"
TITLE_IMAGE = ROOT / "assets" / "yummyimu-product-title.png"


def inline_markup(value: str) -> str:
    parts = re.split(r"(`[^`]*`)", value)
    rendered: list[str] = []
    for part in parts:
        if len(part) >= 2 and part.startswith("`") and part.endswith("`"):
            rendered.append(f"<code>{html.escape(part[1:-1])}</code>")
        else:
            rendered.append(html.escape(part, quote=False))
    return "".join(rendered)


def table_cells(line: str) -> list[str]:
    return [cell.strip() for cell in line.strip().strip("|").split("|")]


def is_table_separator(line: str) -> bool:
    cells = table_cells(line)
    return bool(cells) and all(re.fullmatch(r":?-{3,}:?", cell) for cell in cells)


def markdown_to_html(markdown: str, title_image_uri: str) -> str:
    lines = markdown.splitlines()
    blocks: list[str] = []
    title = "YummyIMU User Manual"
    i = 0

    while i < len(lines):
        line = lines[i].rstrip()

        if line.startswith("<img ") and "YummyIMU" in line:
            i += 1
            continue

        if line.startswith("# "):
            title = line[2:].strip()
            i += 1
            continue

        if not line.strip():
            i += 1
            continue

        if line.startswith("```"):
            i += 1
            code_lines: list[str] = []
            while i < len(lines) and not lines[i].startswith("```"):
                code_lines.append(lines[i])
                i += 1
            if i < len(lines):
                i += 1
            blocks.append(f"<pre>{html.escape(chr(10).join(code_lines))}</pre>")
            continue

        if (
            line.startswith("|")
            and i + 1 < len(lines)
            and lines[i + 1].startswith("|")
            and is_table_separator(lines[i + 1])
        ):
            headers = table_cells(line)
            i += 2
            rows: list[list[str]] = []
            while i < len(lines) and lines[i].lstrip().startswith("|"):
                rows.append(table_cells(lines[i]))
                i += 1
            head = "".join(f"<th>{inline_markup(cell)}</th>" for cell in headers)
            body = "".join(
                "<tr>" + "".join(f"<td>{inline_markup(cell)}</td>" for cell in row) + "</tr>"
                for row in rows
            )
            blocks.append(f"<table><thead><tr>{head}</tr></thead><tbody>{body}</tbody></table>")
            continue

        if line.startswith("## "):
            blocks.append(f"<h2>{inline_markup(line[3:].strip())}</h2>")
            i += 1
            continue

        if line.startswith("### "):
            blocks.append(f"<h3>{inline_markup(line[4:].strip())}</h3>")
            i += 1
            continue

        if line.startswith("- "):
            items: list[str] = []
            while i < len(lines) and lines[i].startswith("- "):
                items.append(f"<li>{inline_markup(lines[i][2:].strip())}</li>")
                i += 1
            blocks.append("<ul>" + "".join(items) + "</ul>")
            continue

        if re.match(r"^\d+\.\s+", line):
            items = []
            while i < len(lines) and re.match(r"^\d+\.\s+", lines[i]):
                item = re.sub(r"^\d+\.\s+", "", lines[i]).strip()
                items.append(f"<li>{inline_markup(item)}</li>")
                i += 1
            blocks.append("<ol>" + "".join(items) + "</ol>")
            continue

        paragraph = [line.strip()]
        i += 1
        while i < len(lines):
            candidate = lines[i].rstrip()
            if not candidate.strip():
                break
            if (
                candidate.startswith(("#", "```", "- ", "|", "<img "))
                or re.match(r"^\d+\.\s+", candidate)
            ):
                break
            paragraph.append(candidate.strip())
            i += 1
        blocks.append(f"<p>{inline_markup(' '.join(paragraph))}</p>")

    content = "\n".join(blocks)
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>{html.escape(title)}</title>
<style>
  @page {{
    size: A4;
    margin: 15mm 18mm 17mm;
    @bottom-right {{
      content: "YummyIMU | " counter(page);
      color: #667085;
      font: 7.5pt "Microsoft YaHei", sans-serif;
    }}
  }}
  * {{ box-sizing: border-box; }}
  html {{ print-color-adjust: exact; -webkit-print-color-adjust: exact; }}
  body {{
    margin: 0;
    color: #182230;
    font-family: "Microsoft YaHei", "Noto Sans CJK SC", sans-serif;
    font-size: 9.35pt;
    line-height: 1.5;
  }}
  .manual-title {{
    min-height: 82pt;
    margin: 0 0 12pt;
    display: flex;
    align-items: center;
    justify-content: space-between;
    gap: 24pt;
  }}
  .manual-title h1 {{
    margin: 0;
    color: #182230;
    font-size: 24pt;
    line-height: 1.15;
    letter-spacing: 0;
  }}
  .manual-title img {{
    display: block;
    width: 58pt;
    height: auto;
    max-height: 82pt;
    object-fit: contain;
    flex: none;
  }}
  h2, h3 {{ color: #182230; break-after: avoid-page; page-break-after: avoid; }}
  h2 {{ margin: 10pt 0 5pt; font-size: 16.2pt; line-height: 1.3; }}
  h3 {{ margin: 7pt 0 4pt; font-size: 11.6pt; line-height: 1.35; }}
  p {{ margin: 0 0 5pt; orphans: 2; widows: 2; }}
  ul, ol {{ margin: 2pt 0 7pt; padding-left: 19pt; }}
  li {{ margin: 0 0 3.2pt; padding-left: 1pt; }}
  code {{
    color: #344054;
    font-family: Consolas, "Microsoft YaHei", monospace;
    font-size: 0.94em;
  }}
  pre {{
    margin: 4pt 0 7pt;
    padding: 6pt 9pt;
    overflow-wrap: anywhere;
    white-space: pre-wrap;
    color: #253347;
    background: #f2f4f7;
    border: 0.35pt solid #c9d0dc;
    font-family: Consolas, "Microsoft YaHei", monospace;
    font-size: 8.15pt;
    line-height: 1.38;
    break-inside: avoid;
  }}
  table {{
    width: 100%;
    margin: 4pt 0 7pt;
    border-collapse: collapse;
    table-layout: auto;
    font-size: 8.1pt;
  }}
  thead {{ display: table-header-group; }}
  tr {{ break-inside: avoid; page-break-inside: avoid; }}
  th, td {{
    padding: 3.2pt 5pt;
    border: 0.35pt solid #c9d0dc;
    text-align: left;
    vertical-align: middle;
    overflow-wrap: anywhere;
  }}
  th {{ color: white; background: #344054; font-weight: 700; }}
  tbody tr:nth-child(even) td {{ background: #f6f8fa; }}
</style>
</head>
<body>
  <div class="manual-title">
    <h1>{html.escape(title)}</h1>
    <img src="{title_image_uri}" alt="YummyIMU product">
  </div>
  {content}
</body>
</html>
"""


def build_title_image() -> None:
    with Image.open(SOURCE_IMAGE) as source:
        image = source.convert("RGBA")
        alpha = image.getchannel("A")
        solid = alpha.point(lambda value: 255 if value >= 8 else 0)
        bbox = solid.getbbox()
        if bbox is None:
            raise RuntimeError(f"No visible pixels found in {SOURCE_IMAGE}")
        padding = 24
        left = max(0, bbox[0] - padding)
        top = max(0, bbox[1] - padding)
        right = min(image.width, bbox[2] + padding)
        bottom = min(image.height, bbox[3] + padding)
        image.crop((left, top, right, bottom)).save(TITLE_IMAGE, optimize=True)


def find_browser() -> Path:
    candidates = (
        Path(r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"),
        Path(r"C:\Program Files\Microsoft\Edge\Application\msedge.exe"),
        Path(r"C:\Program Files\Google\Chrome\Application\chrome.exe"),
        Path(r"C:\Program Files (x86)\Google\Chrome\Application\chrome.exe"),
    )
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise FileNotFoundError("Microsoft Edge or Google Chrome was not found")


def export_pdf(source: Path, output: Path) -> None:
    build_title_image()
    markdown = source.read_text(encoding="utf-8")
    rendered = markdown_to_html(markdown, TITLE_IMAGE.resolve().as_uri())
    output.parent.mkdir(parents=True, exist_ok=True)

    temp_root = ROOT / "tmp" / "pdfs"
    temp_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="manual-export-", dir=temp_root) as temp_dir:
        temp = Path(temp_dir)
        html_path = temp / "USER_MANUAL.html"
        raw_pdf = temp / "USER_MANUAL.raw.pdf"
        profile = temp / "browser-profile"
        html_path.write_text(rendered, encoding="utf-8")

        command = [
            str(find_browser()),
            "--headless=new",
            "--disable-gpu",
            "--disable-extensions",
            "--allow-file-access-from-files",
            "--no-pdf-header-footer",
            f"--user-data-dir={profile}",
            f"--print-to-pdf={raw_pdf}",
            html_path.resolve().as_uri(),
        ]
        completed = subprocess.run(command, capture_output=True, text=True, timeout=90)
        if completed.returncode != 0 or not raw_pdf.exists():
            details = completed.stderr.strip() or completed.stdout.strip()
            raise RuntimeError(f"Browser PDF export failed: {details}")

        reader = PdfReader(raw_pdf)
        writer = PdfWriter()
        for page in reader.pages:
            writer.add_page(page)
        writer.add_metadata(
            {
                "/Title": "YummyIMU User Manual",
                "/Subject": "YummyIMU communication protocol and user guide",
                "/Author": "YummyIMU",
            }
        )
        staged = temp / "USER_MANUAL.final.pdf"
        with staged.open("wb") as stream:
            writer.write(stream)
        shutil.copyfile(staged, output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    export_pdf(args.source.resolve(), args.output.resolve())
    print(f"Exported {args.output.resolve()}")


if __name__ == "__main__":
    main()
