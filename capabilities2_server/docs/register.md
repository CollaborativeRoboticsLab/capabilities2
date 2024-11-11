## Registering a capability

Capabilities can be registered by exporting them in a package. The capabilities2 server will read the capabilities from the package and make them available to the user. Basic capability use case needs two components to be registered; `capability_interface` and `capability_provider`. Optionally, a third component can be added as `semantic_capability_interface`.

```xml
<!-- see std_capabilities package for examples -->
<export>
    <capability_interface>
        interfaces/cool_cap.yaml
    </capability_interface>
    
    <capability_provider>
        providers/cool_cap.yaml
    </capability_provider>

    <semantic_capability_interface>
        semantic_interfaces/not_cool_cap.yaml
    </semantic_capability_interface>
</export>
```

Capabilities can also be registered through a service call. This is useful for registering capabilities that are not exported in a package. The service call has been implemented as a node in the capabilities2 package. Use the node to register capabilities during runtime. This method can also be used to update capabilities.
