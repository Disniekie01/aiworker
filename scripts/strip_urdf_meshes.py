#!/usr/bin/env python3
"""Rewrite URDF: replace mesh/cylinder geometry with tiny boxes for tools without ffw_description meshes."""
import sys
import xml.etree.ElementTree as ET


def main():
    inp, outp = sys.argv[1], sys.argv[2]
    tree = ET.parse(inp)
    root = tree.getroot()
    for link in root.findall("link"):
        for el in list(link):
            if el.tag not in ("visual", "collision"):
                continue
            g = el.find("geometry")
            if g is None:
                continue
            el.remove(g)
            ng = ET.SubElement(el, "geometry")
            ET.SubElement(ng, "box", attrib={"size": "0.01 0.01 0.01"})
    tree.write(outp, encoding="unicode", xml_declaration=True)


if __name__ == "__main__":
    main()
