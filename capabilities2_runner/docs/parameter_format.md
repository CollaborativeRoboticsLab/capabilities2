## Parameter Format

# TODO

Runners can use parameters. These parameters are passed to the runner in the `trigger` function. The parameter format for capabilities2 uses XML. A runner expects to receive a minimal XML structure which includes an ID attribute, for example:

```xml
<parameters id="1234">
```

The `id` is used to identify the runner instance and manage its execution.
