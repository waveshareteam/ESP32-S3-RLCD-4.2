#!/usr/bin/env python3
"""Convert the 400x300 binary PGM files written by the preview tool to PNG.

    python to_png.py [--scale N] [--crop x0,y0,x1,y1] [--stack out.png] file.pgm [file2.pgm ...]

Each input becomes <name>.png next to it.  --crop keeps only that rectangle of every
input (before scaling); --stack puts all the inputs one above the other in a single
out.png instead (handy for comparing a detail across scenarios).  Needs Pillow
(pip install pillow).
"""
import sys
from PIL import Image


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    # header: P5\n<w> <h>\n255\n
    parts = data.split(b"\n", 3)
    assert parts[0] == b"P5", "not a binary PGM"
    w, h = (int(v) for v in parts[1].split())
    return Image.frombytes("L", (w, h), parts[3][: w * h])


def main():
    args = sys.argv[1:]
    scale = 2
    crop = None
    stack = None
    while args and args[0].startswith("--"):
        opt = args.pop(0)
        if opt == "--scale":
            scale = int(args.pop(0))
        elif opt == "--crop":
            crop = tuple(int(v) for v in args.pop(0).split(","))
        elif opt == "--stack":
            stack = args.pop(0)
        else:
            sys.exit("unknown option " + opt)

    images = []
    for path in args:
        img = read_pgm(path).convert("1")
        if crop:
            img = img.crop(crop)
        images.append((path, img))

    if stack:
        gap = 2
        width = max(i.width for _, i in images)
        height = sum(i.height for _, i in images) + gap * (len(images) - 1)
        sheet = Image.new("1", (width, height), 1)
        y = 0
        for _, img in images:
            sheet.paste(img, (0, y))
            y += img.height + gap
        if scale != 1:
            sheet = sheet.resize((sheet.width * scale, sheet.height * scale), Image.NEAREST)
        sheet.save(stack)
        print("wrote", stack)
        return

    for path, img in images:
        if scale != 1:
            img = img.resize((img.width * scale, img.height * scale), Image.NEAREST)
        out = path.rsplit(".", 1)[0] + ".png"
        img.save(out)
        print("wrote", out)


if __name__ == "__main__":
    main()
