#!/usr/bin/python
# mec-turtle-hershey-example - a trivial demo to say hello
# scruss - 2014-05-06 - dual WTFPL (srsly)
# mcross - 2016-04-05 - playing with expanding it

from string import split
import hersheyparse
import hersheymap_parse
import turtle

# config (convert to cmd line eventually)

text = "This is a test"
# default: mapfile = "romans.hmp"
mapfile = "romans.hmp"
fontfile = "hershey-occidental.dat"

# Parse the font file

glyphs = {}
for line in open(fontfile, 'r'):
    glyph = hersheyparse.hersheyparse(line.rstrip())
    glyphs[glyph['charcode']] = glyph

# Parse the character map file

hersheymap = hersheymap_parse.parse(mapfile)

# Given an ascii character, return the hershey glyph object associated with it.
def hersheyglyph(char):
    return glyphs[hersheymap[ord(char) - 32]]

x_scale = 3
y_scale = -3  # remember: Y is +ve *down* for Hershey ...
x = -600
y = 400
turtle.penup()
turtle.pensize(3)
turtle.goto(x, y)
for c in list(text):
    glyph = hersheyglyph(c)
    x_origin = x
    y_origin = y
    for line in glyph['lines']:
        first = 1
        for pt in line:
            if first == 1:
                first = 0
                turtle.goto(x_origin + x_scale * (pt[0] - glyph['left'
                            ]), y_origin + y_scale * pt[1])
                turtle.pendown()
            else:
                turtle.goto(x_origin + x_scale * (pt[0] - glyph['left'
                            ]), y_origin + y_scale * pt[1])
        turtle.penup()

        # don't forget to move to (right sidebearing, 0) at end of draw

        x = x_origin + x_scale * (glyph['right'] - glyph['left'])
        y = y_origin

turtle.done()
