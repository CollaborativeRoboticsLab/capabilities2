# Capability Loading

The capabilities server loads exported capability specs from discovered packages and inserts them into the capabilities database during startup.

## Deferred Loading

Some capability specs depend on other specs already existing in the database. The main case today is semantic interfaces: a semantic interface `redefines` a base interface, and that relationship is enforced by a database foreign key.

Package discovery order is not guaranteed to match dependency order. That means a valid semantic interface can be encountered before the interface it redefines has been loaded. Without special handling, startup would report a foreign key violation even though the missing parent capability may be loaded a little later in the same startup pass.

To handle that, capability loading uses deferred retries:

1. Load each spec normally.
2. If insertion succeeds, keep going.
3. If insertion fails because of a foreign key dependency, defer that spec instead of failing immediately.
4. After the initial pass completes, retry the deferred specs.
5. If a deferred spec succeeds on retry, it is added normally.
6. If no more progress can be made, any still-deferred specs are reported as real errors.

This keeps startup robust when capability packages are discovered in a non-topological order, while still surfacing genuine missing-parent problems once all retries are exhausted.
