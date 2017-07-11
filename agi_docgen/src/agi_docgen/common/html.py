#!/usr/bin/env python
################################################################################
#
# Copyright Airbus Group SAS 2015
# All rigths reserved.
#
# File Name : robot_axis_widget.py
# Authors : Martin Matignon
#
# If you find any bug or if you have any question please contact
# Adolfo Suarez Roos <adolfo.suarez@airbus.com>
# Martin Matignon <martin.matignon.external@airbus.com>
#
#
################################################################################
class Attrib:
    id   = 'id'
    type = 'type'
    href = 'href'

class Metadata:
    head   = "head"
    title  = "title"
    base   = "base"
    link   = "link"
    meta   = "meta"
    style  = "style"

class Sections:
    body     = "body"
    article  = "article"
    section  = "section"
    nav      = "nav"
    aside    = "aside"
    h1       = "h1"
    h2       = "h2"
    h3       = "h3"
    h4       = "h4"
    h5       = "h5"
    h6       = "h6"
    header   = "header"
    footer   = "footer"
    address  = "address"

class Grouping:
    p           = "p"
    pre         = "pre"
    blockquote  = "blockquote"
    ol          = "ol"
    ul          = "ul"
    li          = "li"
    dl          = "dl"
    dt          = "dt"
    dd          = "dd"
    figure      = "figure"
    figcaption  = "figcaption"
    div         = "div"
    main        = "main"
    hr          = "hr"

class Text:
    a      = "a"
    em     = "em"
    strong = "strong"
    cite   = "cite"
    q      = "q"
    dfn    = "dfn"
    abbr   = "abbr"
    data   = "data"
    time   = "time"
    code   = "code"
    var    = "var"
    samp   = "samp"
    kbd    = "kbd"
    mark   = "mark"
    ruby   = "ruby"
    rb     = "rb"
    rt     = "rt"
    rp     = "rp"
    rtc    = "rtc"
    bdi    = "bdi"
    bdo    = "bdo"
    span   = "span"
    br     = "br"
    wbr    = "wbr"
    small  = "small"
    i      = "i"
    b      = "b"
    u      = "u"
    s      = "s"
    sub    = "sub"
    sup    = "sup"

class Edits:
    ins    = "ins"
    delete = "delete" 

class EmbeddedContent:
    img    = "img"
    embed  = "embed"
    object = "object"
    param  = "param"
    video  = "video"
    audio  = "audio"
    source = "source"
    track  = "track"
    map    = "map"
    area   = "area"
    iframe = "iframe"

class Tables:
    table    = "table"
    tr       = "tr"
    td       = "td"
    th       = "th"
    caption  = "caption"
    tbody    = "tbody"
    thead    = "thead"
    tfoot    = "tfoot"
    colgroup = "colgroup"
    col      = "col"

class Forms:
    form     = "form"
    input    = "input"
    textarea = "textarea"
    select   = "select"
    option   = "option"
    optgroup = "optgroup"
    datalist = "datalist"
    label    = "label"
    fieldset = "fieldset"
    legend   = "legend"
    button   = "button"
    output   = "output"
    progress = "progress"
    meter    = "meter"
    keygen   = "keygen"

class Scripting:
    script   = "script"
    noscript = "noscript"
    template = "template"
    canvas   = "canvas"

from xml.etree import ElementTree as ET
from xml.etree.ElementTree import tostring
from xml.etree.ElementTree import parse

HtmlElementTree = ET.ElementTree
HtmlElement = ET.Element

def indent(elem, level=0):
    
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for elem in elem:
            indent(elem, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i

def loadHtml(htmlfile):
    f = parse(htmlfile)
    return f.getroot()

class HTMLException(HtmlElement):
    def __init__(self, ex, parent=None):
        HtmlElement.__init__(self, Grouping.p, attrib={"id":"exception"})
        self.text = str(ex)
        
        if parent is not None:
            parent.append(self)
        
    