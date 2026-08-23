import pymupdf

doc = pymupdf.open("rules.pdf")
for i in range(10, min(20, len(doc))):
    doc[i].get_pixmap(dpi=200).save(f"page_{i+1:02d}.png")
    print(f"Saved page_{i+1:02d}.png")
doc.close()
print("Done")
