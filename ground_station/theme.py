#!/usr/bin/env python3
"""
One design system for the whole ground station — Apple HIG, in Qt terms.

Every colour, type size, corner radius and gap in the GUI comes from here. It
exists because the same four numbers were being re-typed in main.py, topbar.py,
splash.py and inputs_panel.py with slightly different values each time: five
greens, four greys and three "nearly black" backgrounds that no longer matched
after a year of edits. A retheme meant finding all of them. Now it means one
file, and `python3 theme.py` prints the whole palette to check it.

WHAT THIS IS NOT: a repaint of the rig's meaning. The brand navy, the light bar
over dark video, the 40px panel header, the green/red drive jabs and the chip
width floors are all decisions with operator sign-off written next to them in
their own modules. This module supplies the *vocabulary* those decisions are
expressed in; it does not overrule any of them. Where a HIG value and an
operator call disagreed, the operator call won and is noted at that line.

The HIG side of it, for anyone who has not read Apple's guidelines:

  Clarity     every element has a purpose; hierarchy carries the meaning
  Deference   chrome supports content, never competes — here, content is video
  Depth       layers, not outlines, say what sits on top of what
  Consistency one type ramp, one grid, one palette, everywhere

Deference is the one that matters most on this rig. The operator is looking at
a duct, not at us, so the UI's job is to disappear until something is wrong.
That is why the borders got quieter and the status colours did not.

    python3 theme.py          # print the palette and the type ramp
"""

from __future__ import annotations

# --- Typography ---------------------------------------------------------------
# SF Pro is Apple's and cannot ship here, so Inter stands in for it: same
# humanist-grotesque skeleton, same tall x-height, designed for UI at small
# sizes. Crucially it ships in the SAME TWO OPTICAL SIZES Apple uses — Inter
# Display for large type, Inter for body — so the HIG's "Display >= 20pt, Text
# below" rule transfers exactly rather than approximately. font_for() applies it.
#
# Installed with `sudo apt-get install fonts-inter` (Debian trixie, 4.1). The
# fallbacks are the two faces this Pi has always had, so a rig that never got
# the package renders as it did before instead of falling back to a serif.
FAMILY_TEXT = "Inter"
FAMILY_DISPLAY = "Inter Display"
FALLBACKS = ["Noto Sans", "DejaVu Sans", "sans-serif"]

# The optical-size threshold, in px. Apple's is 20pt; Inter Display is drawn for
# the same crossover, so the number carries straight over.
DISPLAY_MIN = 20

# --- the numeric face, and why it is a different one --------------------------
# APPLE'S ADVICE HERE IS UNAMBIGUOUS: any number that changes in place — a
# clock, a percentage, a countdown — must be set in TABULAR figures, or the
# digits change width as they change value and the whole reading twitches. SF
# Pro carries tabular figures and you switch them on with a font feature.
#
# Inter carries them too, under the standard OpenType `tnum` tag. WE CANNOT
# REACH IT. Qt exposes QFont.setFeature(QFont.Tag, value), but this PySide6
# build ships QFont.Tag with no constructor that takes a string or an int:
#
#     QFont.Tag('tnum')   -> TypeError: called with wrong argument types
#     QFont.Tag(1953396077) -> TypeError: same
#     QFont.Tag.fromString -> AttributeError, does not exist
#
# so there is no way to build the tag object the setter needs. The style-sheet
# route (`font-feature-settings: "tnum" 1`) does not help either: it can only
# reach QSS-styled widgets, and half the numbers in this app are drawn with
# QPainter, which never sees a style sheet.
#
# MEASURED, rather than assumed — Inter's default figures really are
# proportional, at PT_VALUE on this Pi:
#
#     Inter          0:10  1:7   2:10  5:9   7:9    -> NOT uniform
#     Inter Display  0:10  1:6   2:9   7:8          -> NOT uniform
#     DejaVu Sans    every digit 10                 -> uniform
#
# A "1" is three pixels narrower than a "0" in Inter, so "THROTTLE 100%"
# and "THROTTLE 111%" are different widths and a centred readout slides.
#
# So the numbers that move keep the face that has uniform figures natively. It
# is the face this app used for everything until now, which means those
# readouts render EXACTLY as they always have — this is not a regression, it is
# the one thing that did not change. Everything else gets Inter.
#
# If a later Qt makes QFont.Tag constructible, the right fix is to delete this
# and set `tnum` on FAMILY_TEXT; the call sites all go through font_numeric(),
# so it is a one-line change here.
FAMILY_NUMERIC = "DejaVu Sans"
NUMERIC_FALLBACKS = ["Noto Sans", "Liberation Sans", "monospace"]

# iOS type ramp. These are the HIG's own sizes — body is 17, and everything else
# is placed relative to it. The old GUI ran on 12px with 18px panel names, which
# is a console ramp, not a UI one: two sizes cannot express four levels of
# hierarchy, so weight and colour were doing work that size should have done.
#
# Read across a control room rather than at arm's length, so nothing below
# CAPTION1 is used for anything an operator must read while driving.
CAPTION2 = 11
CAPTION1 = 12
FOOTNOTE = 13
SUBHEAD = 15
BODY = 17
HEADLINE = 17          # same size as BODY, separated by weight — HIG's own rule
TITLE3 = 20
TITLE2 = 22
TITLE1 = 28
LARGE_TITLE = 34

# Weights, as Qt integers (QFont.Weight values). Named so call sites read as
# design intent rather than as numbers.
W_REGULAR = 400
W_MEDIUM = 500
W_SEMIBOLD = 600
W_BOLD = 700


def family_for(size_px: int) -> str:
    """Inter Display at >= 20px, Inter below. Apple's optical-size rule."""
    return FAMILY_DISPLAY if size_px >= DISPLAY_MIN else FAMILY_TEXT


def stack_for(size_px: int) -> str:
    """CSS/QSS font-family list for `size_px`, optical size first."""
    return ", ".join(f'"{f}"' for f in [family_for(size_px)] + FALLBACKS)


def font_for(size_px: int, weight: int = W_REGULAR, tracking: float = None):
    """QFont at `size_px` with the right optical face and HIG tracking.

    Imported lazily so this module stays importable with no QApplication and no
    display — `python3 theme.py` and the unit checks below both rely on that.

    TRACKING follows Apple's: large type is set tighter, small type looser. It
    is the single cheapest thing that makes non-SF type read as Apple's, and it
    is why the title no longer looks like it was set in a terminal.
    """
    from PySide6.QtGui import QFont

    font = QFont(family_for(size_px), size_px, weight)
    font.setPixelSize(size_px)          # px, not pt — the rig has one fixed DPI
    font.setWeight(QFont.Weight(weight))
    if tracking is None:
        tracking = -0.4 if size_px >= TITLE1 else (-0.2 if size_px >= TITLE3
                                                  else (0.0 if size_px >= SUBHEAD
                                                        else 0.2))
    font.setLetterSpacing(QFont.AbsoluteSpacing, tracking)
    return font


def stack_numeric() -> str:
    """QSS font-family list for figures that must hold still. See FAMILY_NUMERIC."""
    return ", ".join(f'"{f}"' for f in [FAMILY_NUMERIC] + NUMERIC_FALLBACKS)


def font_numeric(size_px: int, weight: int = W_REGULAR):
    """QFont for a number that changes in place — uniform digit widths.

    Tracking is left at zero deliberately: the figures are already monospaced
    by the face, and adding letter spacing to a tabular reading widens every
    digit cell without improving the alignment it exists to protect.
    """
    from PySide6.QtGui import QFont

    font = QFont()
    font.setFamilies([FAMILY_NUMERIC] + NUMERIC_FALLBACKS)
    font.setPixelSize(size_px)
    font.setWeight(QFont.Weight(weight))
    return font


# --- Spacing ------------------------------------------------------------------
# Apple's 8pt grid. Every margin and gap in the GUI is one of these, which is
# what stops the 10px/4px/6px drift that had accumulated across the four UI
# modules. SPACE_1 (4) exists for optical nudges inside a chip, not for layout.
SPACE_1 = 4
SPACE_2 = 8
SPACE_3 = 12
SPACE_4 = 16
SPACE_5 = 20
SPACE_6 = 24
SPACE_8 = 32
SPACE_10 = 40
SPACE_12 = 48

# HIG minimum touch target. There is no touchscreen on the ground station, so
# nothing here is finger-sized for its own sake — but the number is still the
# right floor for a control that has to be hit while the rig is moving, and the
# panel header (40px, operator-pinned) sits just under it deliberately.
TOUCH_MIN = 44

# Height of a chip sitting in a camera panel's 40px header.
#
# PINNED, because a capsule radius is derived from it and QSS renders a
# border-radius LARGER than half a widget's height as a SQUARE - silently, with
# the fill and border still drawn correctly, so it reads as "the radius was
# ignored" rather than "the radius was too big". Left to the layout the tag came
# out ~26px anyway, but "anyway" is not a guarantee and the failure is invisible
# until someone looks closely at a corner.
#
# 26 clears the 13px FOOTNOTE type plus its padding and leaves air either side
# inside the 40px header.
PANEL_TAG_H = 26

# --- Radius -------------------------------------------------------------------
# Apple's ladder, plus the rule that actually makes it look Apple-made:
#
#   CONCENTRICITY — inner_radius + padding = outer_radius
#
# A 12px-radius chip inset 4px inside a container wants a 16px container, not a
# 4px one. Get this wrong and nested corners look like a mistake even when every
# individual radius is defensible. concentric() below does the arithmetic so
# call sites state the relationship instead of hard-coding the answer.
RADIUS_SM = 8
RADIUS_MD = 12
RADIUS_LG = 16
RADIUS_XL = 20
RADIUS_2XL = 24
RADIUS_FULL = 9999          # capsule — QSS clamps to half the height


def concentric(inner_radius: int, padding: int) -> int:
    """Outer radius that keeps a `padding`-inset child visually concentric."""
    return inner_radius + padding


def capsule(height: int) -> int:
    """Radius that makes a `height`-tall widget a true capsule in QSS.

    Qt does not honour a 9999px radius the way a browser does — it needs an
    actual number no larger than half the box. Pass the widget's fixed height.
    """
    return height // 2


# --- Colour -------------------------------------------------------------------
# Two semantic sets, because this app is deliberately two-toned and always has
# been: a LIGHT brand field on the top bar and the splash (carrying the boot
# theme through from Plymouth), over a DARK working field behind the video.
#
# That split is not a HIG violation, it is the HIG's own advice about deference
# taken seriously: video reads best on near-black, and the brand belongs on the
# one strip that is not video. Both sets below are Apple's own light/dark
# system values, so the two halves are now the same palette at two luminances
# rather than two unrelated colour schemes that happened to share a window.

# Brand. Not Apple's, not negotiable — these are the logo's own colours and the
# boot chain (Plymouth -> splash.py -> topbar.py) is painted to match them.
# Everything else on this page is chosen to sit next to these two.
BRAND_INK = "#241f7a"       # the navy of "ARNOBOT"
BRAND_ACCENT = "#3f6fb5"    # the mid blue of the "R"

# iOS system colours — light. Used on the top bar and splash.
LIGHT = {
    "blue":     "#007AFF",
    "green":    "#34C759",
    "indigo":   "#5856D6",
    "orange":   "#FF9500",
    "red":      "#FF3B30",
    "teal":     "#5AC8FA",
    "yellow":   "#FFCC00",

    "gray":     "#8E8E93",
    "gray2":    "#AEAEB2",
    "gray3":    "#C7C7CC",
    "gray4":    "#D1D1D6",
    "gray5":    "#E5E5EA",
    "gray6":    "#F2F2F7",

    # Labels. The HIG's four levels of emphasis, which is what lets the bar
    # drop to one type size without everything shouting equally.
    "label":    BRAND_INK,                  # brand navy stands in for black
    "label2":   "rgba(36, 31, 122, 0.60)",
    "label3":   "rgba(36, 31, 122, 0.30)",
    "label4":   "rgba(36, 31, 122, 0.18)",

    "bg":       "#FFFFFF",
    "bg2":      "#F2F2F7",
    "bg3":      "#FFFFFF",

    "separator":        "rgba(60, 60, 67, 0.29)",
    "separator_opaque": "#C6C6C8",
}

# iOS system colours — dark. Used behind the video and in the inputs strip.
# Apple brightens every hue for dark mode (blue 007AFF -> 0A84FF and so on);
# using the light values on a dark field is the single most common way a dark
# UI ends up looking muddy, and it is what the old palette was doing.
DARK = {
    "blue":     "#0A84FF",
    "green":    "#30D158",
    "indigo":   "#5E5CE6",
    "orange":   "#FF9F0A",
    "red":      "#FF453A",
    "teal":     "#64D2FF",
    "yellow":   "#FFD60A",

    "gray":     "#8E8E93",
    "gray2":    "#636366",
    "gray3":    "#48484A",
    "gray4":    "#3A3A3C",
    "gray5":    "#2C2C2E",
    "gray6":    "#1C1C1E",

    "label":    "#FFFFFF",
    "label2":   "rgba(235, 235, 245, 0.60)",
    "label3":   "rgba(235, 235, 245, 0.30)",
    "label4":   "rgba(235, 235, 245, 0.18)",

    # TRUE BLACK for the video ground, not Apple's #1C1C1E. This is the one
    # place the HIG is deliberately overridden: #000 is what the panel letterbox
    # bars and the NO SIGNAL field want, because any lift at all shows as a grey
    # frame around dark duct footage on an OLED-ish panel. The app chrome around
    # it still uses the system greys, so the video reads as inset into the UI
    # rather than as a hole in it.
    "bg":       "#000000",
    "bg2":      "#1C1C1E",
    "bg3":      "#2C2C2E",

    "separator":        "rgba(84, 84, 88, 0.60)",
    "separator_opaque": "#38383A",
}

# --- Status ------------------------------------------------------------------
# The four states every chip in the app can be in, as (text/dot, border, fill)
# triples, in both fields. Same structure topbar.py's PILL_STATES already used —
# it is a good shape and was kept — but the values are now Apple's system
# semantics instead of six hand-mixed tints.
#
# The mapping is the HIG's: green = success, orange = caution (NOT red — a
# reconnecting camera is not a failed one, and spending red on it means red has
# nothing left to say when the tether actually drops), red = failure, grey =
# nothing to report.
STATUS_LIGHT = {
    "ok":   (LIGHT["green"],  "rgba(52, 199, 89, 0.30)",  "rgba(52, 199, 89, 0.12)"),
    "warn": (LIGHT["orange"], "rgba(255, 149, 0, 0.30)",  "rgba(255, 149, 0, 0.12)"),
    "bad":  (LIGHT["red"],    "rgba(255, 59, 48, 0.30)",  "rgba(255, 59, 48, 0.12)"),
    "idle": (LIGHT["gray"],   "rgba(142, 142, 147, 0.28)", "rgba(142, 142, 147, 0.10)"),
    # The one saturated chip on the bar, unchanged in spirit from the original:
    # every other state is something the operator CHECKS, whereas "am I
    # recording" has to reach them while they are looking at the video. Now in
    # system red rather than a bespoke #d93a33.
    "rec":  ("#FFFFFF",       LIGHT["red"],                LIGHT["red"]),
}

STATUS_DARK = {
    "ok":   (DARK["green"],  "rgba(48, 209, 88, 0.35)",   "rgba(48, 209, 88, 0.15)"),
    "warn": (DARK["orange"], "rgba(255, 159, 10, 0.35)",  "rgba(255, 159, 10, 0.15)"),
    "bad":  (DARK["red"],    "rgba(255, 69, 58, 0.35)",   "rgba(255, 69, 58, 0.15)"),
    "idle": (DARK["gray"],   "rgba(142, 142, 147, 0.35)", "rgba(142, 142, 147, 0.12)"),
    "rec":  ("#FFFFFF",      DARK["red"],                 DARK["red"]),
}

# --- Drive highlight ----------------------------------------------------------
# The jab arrows that say which way the rig is being driven. Kept as RGB tuples
# because VideoCanvas paints them with QPainter rather than QSS.
#
# THESE ARE OPERATOR-SET COLOURS (2026-08-20, third arrangement in one day) and
# the reasoning lives in main.py's CameraPanel — read it before touching this.
# They are listed here only so the whole palette is visible in one place. What
# is worth recording: the back-camera red the operator picked, (255, 59, 48), is
# byte-for-byte iOS systemRed, and the front green is a hotter cousin of
# systemGreen. The pair was already on the Apple ramp before this module
# existed, which is why neither needed changing to fit it.
JAB_FRONT_RGB = (46, 226, 90)
JAB_FRONT_EDGE = (5, 62, 24)
JAB_BACK_RGB = (255, 59, 48)
JAB_BACK_EDGE = (74, 8, 4)

# --- Motion -------------------------------------------------------------------
# Apple's durations, in ms. Qt animations take the same numbers.
DUR_INSTANT = 100       # micro-interaction: a chip changing state
DUR_FAST = 200          # hover / focus
DUR_NORMAL = 300        # standard transition
DUR_SLOW = 500          # splash -> viewer handover

# Apple's standard curve, as the four control points QEasingCurve.BezierSpline
# wants. cubic-bezier(0.25, 0.1, 0.25, 1) — the CSS `ease` default, which is
# what Apple's own default resolves to.
EASE_DEFAULT = (0.25, 0.1, 0.25, 1.0)
EASE_OUT = (0.0, 0.0, 0.58, 1.0)
EASE_SPRING = (0.175, 0.885, 0.32, 1.275)


# --- QSS ----------------------------------------------------------------------
def app_stylesheet() -> str:
    """The whole application's Qt stylesheet, built from the tokens above.

    Replaces the four-rule string that used to sit at the bottom of main.py.
    Structured as HIG layers: the window ground, then cards (panels) that sit on
    it, then the content inside the cards.
    """
    d = DARK
    panel_pad = SPACE_2
    # Concentric: the video canvas is inset SPACE_2 inside the panel, so the
    # panel's own radius has to be the canvas radius plus that inset.
    canvas_radius = RADIUS_MD
    panel_radius = concentric(canvas_radius, panel_pad)

    return f"""
/* --- window ground --------------------------------------------------- */
QWidget {{
    background: {d["bg2"]};
    color: {d["label"]};
    font-family: {stack_for(BODY)};
    font-size: {SUBHEAD}px;
}}

/* --- camera panel: a HIG card, not a bordered box --------------------- */
/* Depth by elevation, not by outline. The old panel drew a 1px #1e2a38 line
   around near-black on near-black, which is a border you can only see if you
   already know it is there. A card that is one step LIGHTER than the window
   ground reads as sitting on top of it at any distance, and costs no pixels. */
#panel {{
    background: {d["bg3"]};
    border: none;
    border-radius: {panel_radius}px;
}}

/* The strip carrying the status dot and the camera name. Height is pinned to
   40px in main.py on the operator's call — NOT set here, deliberately, so this
   file cannot silently override a number that has sign-off. */
#panelHeader {{
    background: transparent;
    border: none;
    border-top-left-radius: {panel_radius}px;
    border-top-right-radius: {panel_radius}px;
}}

/* EVERY LABEL ON THE CARD NEEDS AN EXPLICIT TRANSPARENT GROUND. The QWidget
   rule above paints the whole app in the window colour, and a QLabel is a
   QWidget as far as that rule is concerned — so without this each label sits in
   its own #1C1C1E box on top of the lighter card, which reads as a smudge
   behind the text. topbar.py carries the identical note for the identical
   reason; it is the second half of the same Qt trap as WA_StyledBackground. */
#panelName, #dot {{ background: transparent; }}

/* Headline: HIG separates it from body by WEIGHT at the same size, which is
   what lets the strip stay 40px while the name still reads as a title. The old
   18px + 1px letter-spacing was a console header; tracking now goes slightly
   negative, as Apple sets everything above body size. */
#panelName {{
    color: {d["label"]};
    font-family: {stack_for(HEADLINE)};
    font-size: {HEADLINE}px;
    font-weight: {W_SEMIBOLD};
    letter-spacing: 0.2px;
}}

/* The status dot. Colour is set per-state from STATUS_DARK by main.py — the
   red here is only the at-rest value for a panel that has never connected. */
#dot {{
    font-size: {CAPTION1}px;
    color: {d["red"]};
}}

/* Secondary text anywhere in the app: the HIG's label2, i.e. 60% white. Using
   a dimmer GREY for secondary text (what the old palette did) shifts the hue as
   well as the weight; dropping the alpha keeps it the same colour, quieter. */
.secondary, #secondary {{
    color: {d["label2"]};
    font-size: {FOOTNOTE}px;
}}
""".strip()


def status_qss(state: str, dark: bool = True, height: int = 24) -> str:
    """QSS for one status chip in `state`, as a true capsule.

    Capsule rather than the old 4px rounded rectangle because that is what iOS
    uses for every status affordance it has, and because at this size the corner
    is most of the shape — 4px reads as "rectangle someone softened", 12px reads
    as designed.
    """
    fg, border, fill = (STATUS_DARK if dark else STATUS_LIGHT)[state]
    return (f"color: {fg}; background: {fill}; border: 1px solid {border}; "
            f"border-radius: {capsule(height)}px; "
            f"font-family: {stack_for(FOOTNOTE)}; font-size: {FOOTNOTE}px; "
            f"font-weight: {W_MEDIUM};")


# --- self-check ---------------------------------------------------------------
if __name__ == "__main__":
    def _swatch(name, value):
        print(f"  {name:<20} {value}")

    print(__doc__.strip().splitlines()[0])
    print()
    print("TYPE RAMP")
    for label, size in [("caption2", CAPTION2), ("caption1", CAPTION1),
                        ("footnote", FOOTNOTE), ("subhead", SUBHEAD),
                        ("body", BODY), ("title3", TITLE3), ("title2", TITLE2),
                        ("title1", TITLE1), ("largeTitle", LARGE_TITLE)]:
        print(f"  {label:<12} {size:>3}px   {family_for(size)}")

    print()
    print("BRAND")
    _swatch("ink", BRAND_INK)
    _swatch("accent", BRAND_ACCENT)

    for title, table in (("LIGHT", LIGHT), ("DARK", DARK)):
        print()
        print(title)
        for key in ("blue", "green", "orange", "red", "label", "label2",
                    "bg", "bg2", "bg3", "separator_opaque"):
            _swatch(key, table[key])

    print()
    print("CONCENTRIC CHECK")
    print(f"  canvas {RADIUS_MD} + pad {SPACE_2} = panel {concentric(RADIUS_MD, SPACE_2)}")
    print(f"  capsule(24) = {capsule(24)}   capsule(28) = {capsule(28)}")
