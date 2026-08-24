# fanuc_tp_sim

Practice FANUC TP programming without a controller: registers, DI/DO logic,
UFRAME/UTOOL, pendant key flow, and a cell that faults when the interlocks are
wrong. Pure Python 3, stdlib only, no install. Matplotlib is optional and only
for the 3D view.

## Run it

From the folder containing `fanuc_tp_sim/`:

```
python3 -m fanuc_tp_sim check   programs                 # lint .LS
python3 -m fanuc_tp_sim run     programs MTEND --trace   # headless, with timing
python3 -m fanuc_tp_sim pendant programs                 # teach pendant
python3 -m fanuc_tp_sim view    programs MTEND           # 3D, runs on its own
```

Pendant needs a terminal of at least 100x30. It is the whole screen: status
and joint/XYZWPR on top, program listing left, ASCII top view of the cell
right, cell state on the bottom two lines.

Pendant + 3D together, pendant driving:

```
python3 -m fanuc_tp_sim pendant programs           # terminal 1
python3 -m fanuc_tp_sim view    programs --follow  # terminal 2
```

The pendant broadcasts joint angles on UDP 127.0.0.1:5999 after every tick;
`--follow` only draws what arrives. Keys go to terminal 1.

## Pendant keys

Single keypresses, no Enter.

```
s  SHIFT latch (jog is refused without it)   f  FWD one line
1-6 / qwerty  jog axis + / -                 g  AUTO run
c  coordinate: JOINT WORLD TOOL USER         h  HOLD
+ -  speed override                          k  RESET alarm
v V  next / previous screen                  b  BWD one line
p  TOUCHUP P[] on the cursor line            a  abort to line 1
i  force a DI (on the I/O screen)            e  edit value under cursor
S  save program to .LS                       x  exit
```

First moves: `s`, then `1` a few times — J1 swings. `c` to WORLD, `1` again —
same key, now +X. `f` steps one line. Screen `HELP` lists this in-app.

## What each file does

**`kin.py`** Forward kinematics is a chain of 4x4 transforms. The inverse has
no closed form here, so it is solved by damped least squares: measure the
6-vector error to the target, ask the Jacobian which joint moves shrink it,
take a bounded step. The damping bounds that step when the Jacobian loses rank
— a singularity, and why a real robot throws MOTN-023 near a straight wrist.
Retrying from several postures stands in for FANUC's N/F, U/D, T/B config
flags.

**`lsparse.py`** `.LS` is ASCII: an `/MN` section of pendant lines, a `/POS`
section of taught points. Parse, edit, write back. This is why offline TP work
is possible at all.

**`runtime.py`** One `step(dt)` either executes an instruction or advances an
in-flight motion. Motion is a state, not a blocking call — that is why FWD,
HOLD and override behave correctly. Holds registers, PR[], all I/O, flags,
timers, frame tables, the call stack with `AR[]`, and the alarm.

**`cell.py`** The plant. Gripper 0.4 s, chuck 0.6 s, door 1.5 s, spindle 6 s.
Parts move according to what is actually holding them. Enter the machine with
the door not open, start a cycle with the robot inside, or open the chuck with
nothing holding the part, and you get `SRVO-050`.

**`pendant.py`** The key flow and screens. **`viewer.py`** The 3D view: joint
centres and the tool frame, not CAD geometry. **`ftpsend.py`** Pushes a
verified `.LS` to a controller's FTP (`MD:`, `FR:`, `UD1:`).

## Exercises

1. Jog in JOINT, then `c` to WORLD and jog again — same keys, different frame.
2. On FRAMES set UTOOL_NUM=0 and jog TOOL Z, then back to 1 (200 mm gripper).
   The wrist orbits the TCP instead of the flange. That is what UTOOL is.
3. Jog into the machine (+Y past 250) with the door shut. Read the alarm.
4. `f` through MTEND with the I/O screen open; watch DO lead DI by the
   actuator time.
5. Force `DI[3] chuck closed` OFF with `i` while the program waits on it — the
   TIMEOUT branch fires.
6. Replace `WAIT DI[5:door open]=ON` with `WAIT 0.50(sec)`. Crashes at 100%
   override, passes at 10%. That is why timing-based interlocks survive
   commissioning and die in production.
7. Delete P[6], re-jog to the chuck, `p`, `S`, then `check` and `run --trace`.
8. Rewrite MTEND around `UFRAME_NUM=1` with PR[] offsets instead of eight
   taught points. Moving the fixture then means re-teaching one frame.

## Limits

* Generic 6R geometry sized near an M-10iD/12, not a certified model. Reach and
  singularity locations are indicative.
* `CNT` is parsed but does not blend; `FINE` and `CNT100` follow the same path.
* No KAREL, no iRVision, no collision geometry beyond the door plane and
  station proximity, no `.TP` binary — that still needs `maketp.exe` or the
  ASCII Upload option.
* Instructions covered: J/L motion with Offset/Tool_Offset/Skip, R/PR
  arithmetic, DO/RO/GO/AO/F, WAIT with TIMEOUT, IF, SELECT, JMP/LBL, CALL with
  AR[], FOR/ENDFOR, TIMER, UFRAME/UTOOL, MESSAGE, PAUSE. Anything else raises
  `INTP-311` rather than silently passing.

Order of work on real iron: sim, `check`, FTP, dry run at 10%.
