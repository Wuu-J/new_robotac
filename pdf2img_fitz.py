import fitz  # PyMuPDF

doc = fitz.open("rules.pdf")
print(f"总页数: {len(doc)}")

for i in range(min(10, len(doc))):
    page = doc[i]
    pix = page.get_pixmap(dpi=200)
    pix.save(f"page_{i+1:02d}.png")
    print(f"Saved page_{i+1:02d}.png")

doc.close()
print("Done")
