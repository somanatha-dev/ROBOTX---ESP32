# REAR IR + MULTIPLEXER — HARDWARE INSPECTION CHECKLIST

**For: the person who built and wired the rover.**
**Time needed: about 15–20 minutes. Mostly taking photos.**

You do not need to know any electronics to complete this. Everything here is either
*take a photo*, *copy the text you see printed on a part*, or *follow a wire with your
eyes and say where it ends up*.

---

## READ THIS FIRST — 3 rules

**Rule 1 — "I don't know" is a correct answer.**
If you can't find something, or you're not sure, write **DON'T KNOW**. Please do not
guess and do not look up what it "probably" is. A wrong guess is much worse than a
blank, because firmware written on a guess will silently report sensor readings that
were never actually measured. Blanks are fine. We will work around them.

**Rule 2 — photos beat typing.**
If typing out a wiring list is a pain, just take clear photos and say so. A good photo
answers several questions at once. If a photo and something you typed disagree, we will
trust the photo.

**Rule 3 — you do not need to disconnect or unplug anything.**
Everything here can be read with the rover as it sits. Please don't desolder, unplug
connectors, or pull wires out to read a label. If a label is hidden, say
**"hidden, can't see it"** and move on.

**Safety note:** it's fine to have the rover switched **off** for all of this. Nothing
here requires it to be powered on.

---

## PART A — PHOTOS (4 required)

Tips that make photos actually usable:

- **Good light.** Daylight or a bright lamp. Turn the phone flash **off** if the board
  is shiny — flash usually washes out exactly the tiny printed text we need.
- **Hold still and get close**, then check the photo before moving on: can *you* read
  the small text when you zoom in? If not, we can't either.
- **Chip text is often tiny and low-contrast.** Tilting the board slightly so light
  rakes across the surface usually makes it readable.
- More photos is always better than fewer. Extra angles cost you nothing.

### ☐ PHOTO 1 — The rear IR sensor assembly
The three IR sensors at the back of the rover, all three visible in one shot if you can.
Take it from behind the rover, looking at the back.
*We're trying to see: how the three sensors are arranged and spaced, and what they look
like.*

### ☐ PHOTO 2 — The multiplexer board / chip, markings readable
The small board that the three sensors connect into. **This is the most important
photo.** Get close enough that the printed text on the black chip is readable when
zoomed in. If there is text on the green/blue board itself (silkscreen) as well as on
the chip, capture both — take two photos if needed.
*We're trying to see: the exact part number.*

### ☐ PHOTO 3 — The ESP32 side, showing wires going to the multiplexer
The ESP32 board, showing where the wires from the multiplexer plug into it. The pin
labels printed along the edge of the ESP32 board should be readable.
*We're trying to see: which ESP32 pins those wires land on.*

### ☐ PHOTO 4 — The complete rear sensor wiring
A wider shot showing the whole path: sensors → multiplexer → ESP32, with the wires
visible in between. This is the "overview" shot that ties the other three together.
*We're trying to see: whether anything else sits in the middle of that path.*

---

## PART B — MARKINGS (copy the text exactly)

Copy **exactly** what is printed, including dashes, spaces and any trailing letters or
numbers. Don't clean it up or shorten it. If a line is unreadable or partly rubbed off,
copy the part you can read and mark the rest with `?`.

### ☐ 5. Text printed on the MULTIPLEXER

There may be up to three separate things printed. Please give whichever ones exist:

- **On the black chip itself:** ____________________________________
  *(usually a line or two of small text, e.g. a letter-and-number code)*
- **On the board/PCB (silkscreen):** ____________________________________
  *(often a module name near the edge or on the back)*
- **Any other text, sticker or handwriting:** ____________________________________

If the chip text is too small to read by eye, just say so and rely on Photo 2.

### ☐ 6. Text printed on ONE of the IR sensors

Pick any one of the three — they should be identical. Check **both sides** of the little
board; the marking is often on the back.

- **Text on the sensor board:** ____________________________________
- **Text on any chip on the sensor:** ____________________________________
- **Are all three sensors identical?**  ☐ Yes  ☐ No  ☐ Not sure
  *(If No — please say how they differ, and give the markings for each different one.)*

---

## PART C — WHAT PINS DOES THE MULTIPLEXER HAVE?

Look along the edges of the multiplexer board. The pins (or the holes/pads) almost
always have tiny labels printed next to them. **Just tell us which of these labels you
can actually see printed on the board** — tick what's there.

This single answer tells us the most about what kind of part it is, so please take your
time on it.

### ☐ 7. Tick every label you can see printed on the multiplexer board

| Label to look for | Seen? | If yes, how many? |
|---|---|---|
| **S0** | ☐ | |
| **S1** | ☐ | |
| **S2** | ☐ | |
| **S3** | ☐ | |
| **SIG** | ☐ | |
| **Z** | ☐ | |
| **COM** | ☐ | |
| **EN** *(may also be printed as **E** or **INH**)* | ☐ | |
| **SDA** | ☐ | |
| **SCL** | ☐ | |
| **A0** | ☐ | |
| **A1** | ☐ | |
| **A2** | ☐ | |
| **VCC** *(may be **V**, **+**, **3V3**, or **5V**)* | ☐ | |
| **GND** *(may be **G** or **−**)* | ☐ | |

**☐ Also: are there any OTHER labels printed that aren't in this list?**
Please write them down exactly — they matter:
____________________________________________________________

**☐ And: how are the three sensors' connection points labelled?**
Look at where the sensors plug in. Common labels are `C0 C1 C2 …`, or `Y0 Y1 Y2 …`, or
plain numbers `0 1 2 …`. Which style does yours use, and what is the full range printed
(for example `C0` through `C15`, or `Y0` through `Y7`)?
____________________________________________________________

---

## PART D — WHERE DOES EACH WIRE GO?

This is the "follow the wire with your eyes" part.

### ☐ 8. Which ESP32 pin is each multiplexer wire connected to?

For every wire that leaves the multiplexer board, tell us two things: **which pad it
leaves from** (the label next to it on the mux) and **which pin it arrives at on the
ESP32** (the label printed on the ESP32 board next to that header pin).

Copy the ESP32 pin label **exactly as printed on the board** — it might read `D4`, or
`G4`, or `IO4`, or `GPIO4`, or just `4`. All of those are fine and they're all useful.
Please don't convert or interpret them.

Fill in one line per wire. Wire colour is a helpful extra if you can see it:

```
MUX PAD   ->   ESP32 PIN LABEL        (wire colour)
-------        ----------------        ------------
              ->
              ->
              ->
              ->
              ->
              ->
              ->
```

**If a wire does NOT go to the ESP32** — for example it goes to a power rail, a
breadboard strip, a battery pack, or directly to another board — please write where it
actually goes instead. Those are just as important:

```
MUX PAD   ->   goes to...
-------        ----------
              ->
              ->
```

### ☐ Specifically, please make sure these are covered (if the labels exist on your board):

- **S0, S1, S2, S3** → which ESP32 pin each?
- **SIG / Z / COM** (the single "output" pad) → which ESP32 pin?
- **SDA and SCL** (if present) → which ESP32 pins?
- **EN** → is it connected to **GND**? To an **ESP32 pin** (which one)? Or **nothing at
  all / left empty**? ☐ GND ☐ ESP32 pin: ______ ☐ Nothing ☐ Not sure
- **VCC** → where does it get power from? ☐ ESP32 `3V3` pin ☐ ESP32 `5V`/`VIN` pin
  ☐ Battery/other: ______ ☐ Not sure
- **A0 / A1 / A2** (if present) → are any of them wired to anything, or bridged with a
  blob of solder? ☐ All empty ☐ Some bridged (which: ______) ☐ Not sure

### ☐ 9. Which multiplexer channel is each of the three IR sensors plugged into?

Face the **back** of the rover, as if you were standing behind it looking forward. Then
for each sensor, give the channel label printed on the mux where its wire connects:

| Sensor position (viewed from behind) | Mux channel it plugs into |
|---|---|
| **LEFT** sensor | |
| **CENTRE** sensor | |
| **RIGHT** sensor | |

☐ Tick here if you're not sure which channel is which, and we'll work it out by testing
the sensors one at a time instead. **This is a perfectly fine answer** — don't guess.

---

## PART E — POWER AND SIGNAL TYPE

These last three are the ones that matter most for not damaging anything.

### ☐ 10. What voltage do the IR sensors run on?

Follow the power wire (often red) from one of the IR sensors back to where it gets power.

☐ ESP32 pin labelled **3V3** (or **3.3V**)
☐ ESP32 pin labelled **5V** (or **VIN**)
☐ A separate battery pack or power module — please describe: ______________________
☐ It gets power through the multiplexer board, not directly
☐ Not sure

**If there's a marking printed on the sensor board saying a voltage range** (something
like `3.3–5V`), please copy it here: ______________________

### ☐ 11. Is there anything between the sensors/multiplexer and the ESP32?

Look along the wire path in Photo 4. Is there any **extra small board** in the middle —
something the wires pass *through* on the way to the ESP32?

☐ No, the wires go straight from the multiplexer to the ESP32
☐ Yes, there's another small board in between — **please photograph it**
☐ Not sure

If yes, does that extra board have pins labelled **HV** and **LV**, or two matching rows
of pins facing each other? ☐ Yes ☐ No ☐ Can't tell

*(This is asking about a level shifter. If there isn't one, that's a completely normal
answer — we just need to know either way.)*

### ☐ 12. Are the IR sensors on/off, or do they give a varying reading?

Easiest way to tell, by eye:

☐ **Each sensor has a small blue or white screw on it** that can be turned with a tiny
  screwdriver *(this is a sensitivity adjuster)*
☐ **Each sensor has a small LED on it that switches on and off** when you move your hand
  close to it
☐ **Neither of those** — the sensor is just a lens/emitter pair with no adjuster and no
  indicator light
☐ Not sure

**How many wires come out of each IR sensor?**  ☐ 2  ☐ 3  ☐ 4  ☐ Other: ____

**Are the sensor's wires labelled?** If so, copy the labels exactly (for example
`VCC GND OUT`, or `VCC GND DO AO`): ______________________

---

## PART F — THE FILL-IN TABLE

Please fill in what you can. **Leave a cell blank or write `?` rather than guessing.**

| Component | Marking | ESP32 GPIO | Mux channel | Voltage | Output type | Notes |
|---|---|---|---|---|---|---|
| Multiplexer (chip) | | — | — | | — | |
| Multiplexer (board) | | — | — | | — | |
| Mux pin S0 | — | | — | — | — | |
| Mux pin S1 | — | | — | — | — | |
| Mux pin S2 | — | | — | — | — | |
| Mux pin S3 | — | | — | — | — | |
| Mux pin SIG / Z / COM | — | | — | — | — | |
| Mux pin EN | — | | — | — | — | |
| Mux pin SDA | — | | — | — | — | |
| Mux pin SCL | — | | — | — | — | |
| Mux pin A0 / A1 / A2 | — | | — | — | — | |
| Mux VCC | — | | — | | — | |
| Mux GND | — | | — | — | — | |
| IR sensor — LEFT | | | | | | |
| IR sensor — CENTRE | | | | | | |
| IR sensor — RIGHT | | | | | | |
| Level shifter (if any) | | | — | | — | |
| Anything else in the path | | | | | | |

**How to fill each column:**

- **Marking** — the exact text printed on the part. Blank if there's none or you can't
  read it.
- **ESP32 GPIO** — the pin label printed on the ESP32 board (`D4`, `GPIO4`, `IO4`, `4` —
  whatever it actually says). Write `n/a` if that wire doesn't go to the ESP32, and say
  where it goes in Notes.
- **Mux channel** — the channel label the sensor plugs into (`C0`, `Y3`, `2`, …).
- **Voltage** — `3.3V`, `5V`, or where it gets power from.
- **Output type** — `on/off`, `varying`, or `?`. Use Part E question 12 to decide; `?`
  is completely fine.
- **Notes** — anything odd: a loose wire, a repair, a part swapped at some point, two
  wires sharing a pin, something you were never sure about. **Odd details are useful,
  not embarrassing.** Please include them.

---

## DONE — WHAT TO SEND BACK

☐ Photo 1 — rear IR sensor assembly
☐ Photo 2 — multiplexer board/chip, markings readable
☐ Photo 3 — ESP32 side showing wires to the multiplexer
☐ Photo 4 — complete rear sensor wiring
☐ Part B — the exact markings (items 5 and 6)
☐ Part C — which pin labels exist (item 7)
☐ Part D — the wiring list and channel map (items 8 and 9)
☐ Part E — voltage, level shifter, output type (items 10, 11, 12)
☐ Part F — the filled-in table

**If you only have time for three things, do these:** Photo 2, Photo 3, and question 10
(the sensor voltage). Those three unblock the most.

**One last thing that could save everything else:** if you still have the **shop link,
order confirmation, or product listing** for the multiplexer and the IR sensors, send
that too. A product page often answers items 5, 6, 10 and 12 in one go, and it's more
reliable than reading tiny text off a chip.
