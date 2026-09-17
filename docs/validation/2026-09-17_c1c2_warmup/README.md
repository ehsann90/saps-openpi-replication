# C1-C2-W physical warm-up validation — 2026-09-17


Accepted physical validation run:

```text
outputs/physical_c1c2_warmup/c1c2w_20260917T073653Z
```

C1-C2-W validates a one-time, seed-separated, discarded π0.5/DROID
warm-up inference per physical experiment-process invocation. The warm-up
performs no robot or gripper actuation. After warm-up completion, the runner
requires a new physical observation whose wrist-camera, exterior-camera,
arm-state, and gripper-state source timestamps all strictly postdate the
warm-up response-completion barrier.

This milestone does not validate C1-C2 policy-action execution.

## Provenance
```text
SAPS base commit:
35fcecc0239640cee9fc5be59c2fc46767318b77
Validate physical C1-C1 live inference hold

branch:
experiment/physical-fr3-saps

OpenPI:
15a9616a00943ada6c20a0f158e3adb39df2ccac

fr3_lab_stack:
9e535b665626cf8f3894fc4d2ef9136790abad92
```

The SAPS repository was dirty only with the reviewed C1-C2-W implementation under validation.

## Startup contract validated
```text
fresh physical observation
        |
        v
one warm-up inference
  seed = 20260917
  replan_index = 0
  audit_model_input = false
        |
        v
validate finite native [15,8] response
        |
        v
discard complete returned chunk
        |
        v
zero arm commands
zero gripper commands
        |
        v
warm-up response-completion source-time barrier
        |
        v
acquire a new observation with all four source stamps > barrier
        |
        v
prepare real episode
  seed = 20260827
  replan_index = 0
```

Warm-up and real episode sampling therefore remain in separate deterministic
seed domains. The warm-up does not consume the real episode's replan sequence.

## Warm-up evidence
```text
warm-up requests:                  1
warm-up policy seed:               20260917
warm-up replan index:              0
audit_model_input:                 false

returned action shape:             [15, 8]
response discarded:                true

target publications:               0
policy actions executed:           0
gripper commands issued:           0
robot-command publishers:          0
robot services called:             0
robot actions called:              0

excluded from episode timing:      true
```

The process terminated successfully after validating the warm-up and acquiring
the new post-warm-up observation.

## Warm-up timing

For this accepted run:
```text
client round trip:                 0.118882321 s
server inference:                  113.034921 ms
policy inference:                   91.197722 ms

observation age at request:         0.165607452 s
observation age at response:        0.284646273 s
```

This request was already warm because the persistent policy server had serviced
the preceding validation run. It is therefore evidence that the explicit
startup warm-up remains inexpensive when the server is already warm; it is not
used to quantify cold-start reduction.

A preceding validation run observed approximately 3.23 s first-request
round-trip latency, motivating this explicit one-time startup warm-up.

## Post-warm-up source-time barrier

Warm-up response-completion ROS time:

```text
1789630620.3672678
```

Accepted post-warm-up source timestamps:
```text
wrist image:       1789630620.3828363
exterior image:    1789630620.3923790
joint state:       1789630620.4256800
gripper state:     1789630620.4142838
```

All four timestamps strictly exceed the response-completion barrier.

Margins above the barrier were approximately:
```text
wrist image:        15.5685 ms
exterior image:     25.1112 ms
joint state:        58.4122 ms
gripper state:      47.0160 ms
```

The accepted observation remained within the existing freshness contract:
```text
maximum source age:
  wrist image       43.907 ms
  exterior image    34.364 ms
  joint state        1.063 ms
  gripper state     12.460 ms

cross-source skew:  42.844 ms
```

The runner recorded an intermediate rejection with:
```text
Source timestamps must exceed warm-up response barrier
```

before accepting the final observation. This confirms that source-time
advancement was actively enforced rather than only recorded afterward.

## Real-episode continuity

The prepared post-warm-up request retained:
```text
policy episode seed:   20260827
request index:         0
chunk index:           0
replan index:          0
policy actions executed: 0
```

Thus the discarded warm-up did not consume or modify the real episode's first
sampling index.

## Scope and limitations

Validated by C1-C2-W:

- exactly one explicit warm-up request in the validation process;
- independent deterministic warm-up and experiment seeds;
- native finite [15,8] warm-up response;
- complete warm-up response discarded;
- zero robot and gripper actuation;
- warm-up excluded from episode timing and replan accounting;
- post-warm-up acquisition waits for every observation source to cross the warm-up completion barrier;
- the real episode remains at the main seed and replan_index = 0.

Not validated by C1-C2-W:

- first real post-warm-up inference latency in the same process;
- execution of any returned π0.5 action;
- eight-action 15 Hz live policy execution;
- terminal hold following live action execution;
- repeated replanning;
- task success or policy performance.

Those remain C1-C2 validation objectives.
```text
Archived evidence
run.json
warmup_response.json
post_warmup_request.json
SHA256SUMS
```

The JSON evidence files are copied byte-for-byte from the accepted physical run.