from pdf2image import convert_from_path

pages = convert_from_path("rules.pdf", first_page=1, last_page=10, dpi=200)
for i, p in enumerate(pages):
    p.save(f"page_{i+1:02d}.png", "PNG")
    print(f"Saved page_{i+1:02d}.png")
print("Done")
