from pypdf import PdfReader

reader = PdfReader("rules.pdf")
print(f"总页数: {len(reader.pages)}")

print("\n" + "="*40)
print("前20页内容")
print("="*40)

for i, page in enumerate(reader.pages[:20]):
    text = page.extract_text()
    if text:
        print(f"\n--- 第{i+1}页 ---")
        print(text)
    else:
        print(f"\n--- 第{i+1}页 (无文本) ---")
