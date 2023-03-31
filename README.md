# PID

PID Library for the 6TRON ecosystem.

<!-- Describe `PID` library here -->

## Usage

First, create a set of parameters for your PID:

```cpp
sixtron::PID_params pid_params;

pid_params.Kp = 250.0f;
pid_params.Ki = 32.0f;
pid_params.Kd = 0.00f;
```

Then create your PID object using previous parameters, and the delta time at which the PID will be regularly called. Example for a 10ms PID:

```cpp
sixtron::PID my_pid(pid_params, 0.010f)
```

Finally, in your code, define a function that should be called at the defined PID frequency:

```cpp
sixtron::PID_args pid_args;

// Update inputs
pid_args.actual = measured_value;
pid_args.target = target_value;

// compute
my_pid.compute(&pid_args);

// get PID output
my_command = pid_args.output;
```
