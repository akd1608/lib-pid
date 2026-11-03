# PID controller for Zephyr

Zephyr module with a PID controller in classic and velocity ("speed up") form.
Tested with Zephyr 4.3.

*Deutsch: siehe unten.*

## Use

Add the module to your `west.yml`:

```yaml
- name: pid_controller
  path: modules/pid
  revision: v1.1.0
  url: ssh://git@github.com/akd1608/lib-pid.git
```

Enable it in `prj.conf`:

```ini
CONFIG_PID_CONTROLLER=y
```

```c
#include <pid_controller.h>

/* pid_init() does not clear the struct, so it must start zeroed. */
static pid_controller_t pid;
double output;

pid_init(&pid, PID_TYPE_CLASSIC);
pid_tuneParam(&pid, kp, ki, sample_time_ms);
pid_tuneDiffParam(&pid, kd, 0.0, 0.0);
pid_setOutputConstrain(&pid, 0.0, 100.0);
pid_setSumConstrins(&pid, -100.0, 100.0);
pid_setSetPoint(&pid, setpoint);

pid_run(&pid, measurement, &output);
```

If a limit has the same minimum and maximum, that limit is off.

The integral and the derivative use the real time between two `pid_run()` calls, in milliseconds.
The first call and a call after a pause of more than `CONFIG_PID_SAMPLE_GAP_FACTOR` regular steps
count as one regular step. Tune `ki` and `kd` for milliseconds: with `ki` = 0.007, an error of 1 for
one second adds 7 to the output.

## Changes

- **1.1.0:** Fixed the classic form. The integral now adds up (before, each run overwrote it), and the
  time step is the real time since the last run (before, it was the time since start). Values tuned
  on 1.0.0 do not carry over.
- **1.0.0:** First release.

## Options

| Option | Default | Meaning |
|---|---|---|
| `CONFIG_PID_CONTROLLER_FLUSH_BY_NEW_SETPOINT` | off | Clear the error history on a new setpoint |
| `CONFIG_PID_FILTER_COEFFICIENT_ALPHA` | 0 | Derivative weighting, multiplied by 1000 |
| `CONFIG_PID_CONTROLLING_TIME_DEFAULT` | 100 | Not used by the library |
| `CONFIG_PID_FILTER_CONSTRAIN_SUM` | 1000000 | Not used by the library |

## Tests

```sh
west build -b native_sim/native/64 modules/pid/tests/pid -t run
```

## License

PolyForm Noncommercial 1.0.0, see `LICENSE` and `NOTICE.md`. Rights holder: akd
(akd1608@mailbox.org).

---

# PID-Regler für Zephyr

Zephyr-Modul mit einem PID-Regler in klassischer und in Geschwindigkeitsform („speed up“).
Getestet mit Zephyr 4.3. Die Einbindung steht oben im Abschnitt *Use*.

Integral und Differenzial rechnen mit der echten Zeit zwischen zwei `pid_run()`-Aufrufen in
Millisekunden. Der erste Aufruf und ein Aufruf nach einer längeren Pause zählen als ein normaler
Schritt. Mit 1.0.0 eingestellte Werte gelten nicht mehr.

`pid_init()` setzt die Struktur nicht zurück, sie muss also genullt beginnen. Sind Minimum
und Maximum einer Grenze gleich, ist diese Grenze aus.

Tests: `west build -b native_sim/native/64 modules/pid/tests/pid -t run`

Lizenz: PolyForm Noncommercial 1.0.0, siehe `LICENSE` und `NOTICE.md`. Rechteinhaber: akd
(akd1608@mailbox.org).
