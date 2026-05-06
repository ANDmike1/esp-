#!/usr/bin/env python3

"""
NextBoard 硬件方案 Markdown → PDF 转换脚本 (WeasyPrint版)
用法: python md_to_pdf.py input.md output.pdf [--title "方案标题"] [--author "作者"]
      python md_to_pdf.py --merge docs/hardware/ output.pdf [--theme blue]
依赖: pip install weasyprint markdown
"""

import sys
import os
import re
import argparse
import markdown

# ── 配色方案 ──

THEMES = {
    "green": {
        "name": "工程翠绿",
        "primary": "#0F766E",
        "h2": "#1e8449",
        "h3": "#2e86c1",
        "h4": "#5b2c6f",
        "body_color": "#2c3e50",
        "subtitle_color": "#95a5a6",
        "bold_color": "#1a252f",
        "blockquote_bg": "#f8f9fa",
        "blockquote_color": "#5d6d7e",
        "code_bg": "#fdf2e9",
        "code_color": "#c0392b",
        "pre_bg": "#f4f6f7",
        "pre_color": "#2c3e50",
        "pre_border": "",
        "pre_radius": "3pt",
        "thead_bg": "#0F766E",
        "td_border": "#bdc3c7",
        "even_row_bg": "#f8f9fa",
        "hr_color": "#bdc3c7",
        "link_color": "#2e86c1",
        "link_decoration": "none",
        "footer_counter": '"第 " counter(page) " 页"',
    },
    "blue": {
        "name": "商务深蓝",
        "primary": "#1B4F72",
        "h2": "#21618C",
        "h3": "#2E86C1",
        "h4": "#5DADE2",
        "body_color": "#2c3e50",
        "subtitle_color": "#95a5a6",
        "bold_color": "#1a252f",
        "blockquote_bg": "#f8f9fa",
        "blockquote_color": "#5d6d7e",
        "code_bg": "#f0f3f5",
        "code_color": "#c0392b",
        "pre_bg": "#1e1e1e",
        "pre_color": "#d4d4d4",
        "pre_border": "border: 1px solid #333;",
        "pre_radius": "4pt",
        "thead_bg": "#1B4F72",
        "td_border": "#bdc3c7",
        "even_row_bg": "#f8f9fa",
        "hr_color": "#bdc3c7",
        "link_color": "#2E86C1",
        "link_decoration": "none",
        "footer_counter": '"第 " counter(page) " 页 / 共 " counter(pages) " 页"',
    },
    "gray": {
        "name": "极简石墨灰",
        "primary": "#212121",
        "h2": "#424242",
        "h3": "#616161",
        "h4": "#757575",
        "body_color": "#212121",
        "subtitle_color": "#757575",
        "bold_color": "#000000",
        "blockquote_bg": "#f5f5f5",
        "blockquote_color": "#616161",
        "code_bg": "#f5f5f5",
        "code_color": "#d32f2f",
        "pre_bg": "#1e1e1e",
        "pre_color": "#d4d4d4",
        "pre_border": "border: 1px solid #333;",
        "pre_radius": "4pt",
        "thead_bg": "#212121",
        "td_border": "#e0e0e0",
        "even_row_bg": "#fafafa",
        "hr_color": "#bdbdbd",
        "link_color": "#424242",
        "link_decoration": "underline",
        "footer_counter": '"第 " counter(page) " 页 / 共 " counter(pages) " 页"',
    },
}

# ── CSS 模板（用 {key} 占位，由配色方案填充）──
CSS_TEMPLATE = """
@page {{
    size: A4;
    margin: 25mm 20mm 20mm 20mm;

    @top-center {{
        content: "HEADER_TEXT";
        font-family: "Droid Sans Fallback", "Noto Sans CJK SC", Helvetica, Arial, sans-serif;
        font-size: 8pt;
        color: #95a5a6;
        border-bottom: 0.5pt solid #ecf0f1;
        padding-bottom: 3mm;
    }}

    @bottom-center {{
        content: {footer_counter};
        font-family: "Droid Sans Fallback", "Noto Sans CJK SC", Helvetica, Arial, sans-serif;
        font-size: 8pt;
        color: #95a5a6;
        border-top: 0.8pt solid {primary};
        padding-top: 2mm;
    }}
}}

@page :first {{
    @top-center {{ content: none; }}
    @bottom-center {{ content: none; }}
}}

body {{
    font-family: "Droid Sans Fallback", "Noto Sans CJK SC", Helvetica, Arial, sans-serif;
    font-size: 10.5pt;
    line-height: 1.75;
    color: {body_color};
    text-align: justify;
}}

.cover {{ page-break-after: always; text-align: center; padding-top: 40%; }}
.cover h1 {{ font-size: 26pt; color: {primary}; margin-bottom: 8mm; font-weight: bold; letter-spacing: 2pt; }}
.cover .subtitle {{ font-size: 14pt; color: {subtitle_color}; margin-bottom: 6mm; }}
.cover .meta {{ font-size: 11pt; color: {subtitle_color}; margin-bottom: 4mm; }}
.cover .divider {{ width: 60%; margin: 8mm auto; border: none; border-top: 1.5pt solid {primary}; }}

h1 {{ font-size: 20pt; color: {primary}; margin-top: 16mm; margin-bottom: 6mm; padding-bottom: 3mm; border-bottom: 2pt solid {primary}; page-break-before: always; font-weight: bold; }}
h2 {{ font-size: 14pt; color: {h2}; margin-top: 10mm; margin-bottom: 5mm; font-weight: bold; }}
h3 {{ font-size: 12pt; color: {h3}; margin-top: 6mm; margin-bottom: 3mm; font-weight: bold; }}
h4 {{ font-size: 11pt; color: {h4}; margin-top: 5mm; margin-bottom: 2mm; font-weight: bold; }}

p {{ margin-top: 1.5mm; margin-bottom: 1.5mm; orphans: 3; widows: 3; }}
blockquote {{ margin: 4mm 0; padding: 4mm 4mm 4mm 10mm; background: {blockquote_bg}; border-left: 3pt solid {primary}; color: {blockquote_color}; font-size: 10pt; }}
blockquote p {{ margin: 1mm 0; }}
strong, b {{ font-weight: bold; color: {bold_color}; }}

code {{ font-family: "Courier New", Courier, monospace; background: {code_bg}; color: {code_color}; padding: 0.5mm 1.5mm; border-radius: 2pt; font-size: 9.5pt; }}
pre {{ background: {pre_bg}; color: {pre_color}; padding: 4mm; border-radius: {pre_radius}; font-size: 9pt; line-height: 1.5; overflow-wrap: break-word; white-space: pre-wrap; {pre_border} }}
pre code {{ background: none; color: inherit; padding: 0; }}

table {{ width: 100%; border-collapse: collapse; margin: 4mm 0; font-size: 9.5pt; }}
thead th {{ background: {thead_bg}; color: white; padding: 3mm; text-align: left; font-weight: bold; }}
tbody td {{ padding: 2.5mm 3mm; border-bottom: 0.5pt solid {td_border}; }}
tbody tr:nth-child(even) {{ background: {even_row_bg}; }}

hr {{ border: none; border-top: 0.5pt solid {hr_color}; margin: 4mm 0; }}
ul, ol {{ margin: 2mm 0; padding-left: 8mm; }}
li {{ margin-bottom: 1mm; }}
a {{ color: {link_color}; text-decoration: {link_decoration}; }}
"""


def get_css(theme_name):
    """根据配色方案名称生成 CSS"""
    theme = THEMES.get(theme_name, THEMES["green"])
    return CSS_TEMPLATE.format(**theme)


def md_to_html(md_text, title="硬件方案报告", subtitle="NextBoard Hardware Solution",
               meta_line="", author="", theme="green"):
    """将 Markdown 转为带封面的 HTML"""

    html_body = markdown.markdown(
        md_text,
        extensions=['tables', 'fenced_code', 'nl2br'],
        output_format='html5'
    )

    first_h1_match = re.search(r'<h1>(.*?)</h1>', html_body)
    if first_h1_match:
        extracted_title = first_h1_match.group(1)
        if not title or title == "硬件方案报告":
            title = extracted_title
        html_body = html_body.replace(first_h1_match.group(0), '', 1)

    css = get_css(theme).replace("HEADER_TEXT", f"{title} | NextBoard Hardware Solution")

    author_line = f'<div class="meta">作者: {author}</div>' if author else ""
    cover_html = f"""
    <div class="cover">
        <h1 style="page-break-before: avoid; border: none;">{title}</h1>
        <div class="subtitle">{subtitle}</div>
        {"<div class='meta'>" + meta_line + "</div>" if meta_line else ""}
        <hr class="divider">
        {author_line}
    </div>
    """

    full_html = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <style>{css}</style>
</head>
<body>
{cover_html}
{html_body}
</body>
</html>"""

    return full_html


def merge_markdown_files(directory):
    """按文件名顺序合并目录下所有 .md 文件"""
    md_files = sorted(
        [f for f in os.listdir(directory) if f.endswith('.md')],
        key=lambda x: x
    )
    if not md_files:
        print(f"[ERROR] 目录 {directory} 下没有找到 .md 文件")
        sys.exit(1)

    merged = []
    for f in md_files:
        filepath = os.path.join(directory, f)
        with open(filepath, 'r', encoding='utf-8') as fh:
            content = fh.read().strip()
            merged.append(content)
        print(f"  合并: {f}")

    return "\n\n---\n\n".join(merged)


def main():
    theme_names = ", ".join(f"{k} ({v['name']})" for k, v in THEMES.items())
    parser = argparse.ArgumentParser(description="NextBoard 硬件方案 Markdown → PDF")
    parser.add_argument("input", help="输入的 Markdown 文件路径或目录（配合 --merge）")
    parser.add_argument("output", help="输出的 PDF 文件路径")
    parser.add_argument("--title", default=None, help="报告标题")
    parser.add_argument("--author", default="小智学长", help="作者名")
    parser.add_argument("--theme", default="green", choices=THEMES.keys(),
                        help=f"配色方案: {theme_names}（默认 green）")
    parser.add_argument("--merge", action="store_true",
                        help="合并目录下所有 .md 文件为一个 PDF")
    args = parser.parse_args()

    if args.merge:
        if not os.path.isdir(args.input):
            print(f"[ERROR] --merge 模式下 input 必须是目录: {args.input}")
            sys.exit(1)
        print(f"[INFO] 合并目录: {args.input}")
        md_text = merge_markdown_files(args.input)
    else:
        if not os.path.isfile(args.input):
            print(f"[ERROR] 文件不存在: {args.input}")
            sys.exit(1)
        with open(args.input, "r", encoding="utf-8") as f:
            md_text = f.read()

    meta_line = ""
    for line in md_text.split("\n"):
        stripped = line.strip().lstrip(">").strip()
        if "项目" in stripped or "日期" in stripped or "版本" in stripped:
            meta_line = stripped
            break

    title = args.title or "硬件方案报告"
    print(f"[INFO] 配色方案: {THEMES[args.theme]['name']}")
    html = md_to_html(md_text, title=title, meta_line=meta_line,
                      author=args.author, theme=args.theme)

    html_path = args.output.replace('.pdf', '.html')
    with open(html_path, 'w', encoding='utf-8') as f:
        f.write(html)
    print(f"[OK] HTML 已生成: {html_path}")

    try:
        from weasyprint import HTML
    except ImportError:
        print("[ERROR] 缺少依赖，请安装: pip install weasyprint markdown")
        sys.exit(1)

    HTML(string=html).write_pdf(args.output)
    size_kb = os.path.getsize(args.output) / 1024
    print(f"[OK] PDF 已生成: {args.output} ({size_kb:.1f} KB)")


if __name__ == "__main__":
    main()
