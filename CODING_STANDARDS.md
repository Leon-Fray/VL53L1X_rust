# Embedded Rust Coding Standards & Style Guide: Factory-Grade Components

Creating a standard "clean embedded crate" is a good start, but deploying components in an AI-native, cloud-connected factory requires elevating the codebase to a **factory-grade system component**. 

The following architectural and coding standards define how to build mission-critical hardware drivers for Adom Industries, focusing on determinism, fault tolerance, and system-level observability.



## 0. Platform & Portability Requirements

- All crates MUST use `#![no_std]`
- No heap allocation unless explicitly feature-gated
- All hardware access MUST go through:
  - `embedded-hal`
  - `embedded-hal-async`

## 1. Explicit Reliability & Fault Model
Undefined failure behavior leads to system halts. A driver must explicitly define what happens during hardware lock-ups, bus timeouts, or invalid sensor states.

- Panics (`unwrap`, `expect`) are strictly forbidden in library code

- **Custom Error Types**: Use a strongly typed `Error` enum rather than relying purely on HAL bus errors.
  ```rust
  pub enum Error<E> {
      Bus(E),
      Timeout,
      InvalidData,
      SensorFault,
      NotReady,
  }
  ```
- **Recovery & Retry Strategies**: Have defined strategies for when `I2C`/`SPI` fails. Do not use implicit delays. Use explicit timeouts for all operations.

## 2. Async "Native-First" Design
Modern production systems expect `async` as a first-class citizen, not an afterthought. 

- All I/O operations MUST have an async variant
- Async APIs are the canonical implementation
- Blocking APIs MUST NOT duplicate logic

- **Async Core**: Instead of writing blocking code and wrapping it under async feature flags, the core logic should be written async-first.
- **Blocking as a Thin Wrapper**: For generic compatibility, synchronous APIs should serve as thin wrappers or be driven by a small, localized executor over the async core logic, reducing maintenance burden and ensuring the async logic is canonical.

## 3. Concurrency & Multi-Device Strategy
Factories don't use single sensors; they orchestrate arrays of dozens or hundreds.

- Drivers MUST accept bus interfaces via ownership or borrowing
- Drivers MUST NOT internally construct or assume bus instances

- **Zero Global State**: Never use global mutable state (like `static mut` or generic `Mutex` inside the driver).
- **Bus Sharing & Conflict Resolution**: Explicitly design for the `embedded-hal` bus sharing patterns (`SpiDevice` / `I2cDevice`) to handle multi-device occupancy.
- **Orchestrated Resets**: Account for parallel operations, coordinated resets (e.g., XSHUT pins), and shared address conflict management.

## 4. Observability & Diagnostics Layer
In production, "Why did it fail?" is vastly more important than "Does it work?".

- Logging MUST be zero-cost when disabled (feature-gated)

- **Defmt / Log Integration**: Utilize `defmt` (via `#[cfg_attr(feature = "defmt", derive(defmt::Format))]`) or the `log` crate for zero-cost logging in production.
- **Debug Hooks**: Expose the ability to safely query raw status registers directly without mutating state. You cannot fix what you cannot observe. 

## 5. Testing Strategy & Fault Injection
Drivers cannot rely purely on hardware-in-the-loop (HIL) tests. The crate must enforce strict unit testing expectations:

- All tests MUST pass in CI before release
- CI must include:
  - unit tests
  - feature combinations (async / blocking)

- **Mocked HAL Tests**: Use crates like `embedded-hal-mock` to test successful bus transactions purely in software.
- **Fault Injection Tests**: Explicitly simulate bad bus reads, unexpected interrupts, or out-of-order bytes to verify the driver returns the correct `Error` rather than panicking.
- **Register Validation Tests**: Ensure struct mappings and bitfields pack/unpack exactly to and from expected hex structures.

## 6. Timing & Performance Guarantees
Factories rely on strict control loops.

- All operations MUST have a known upper time bound
- Drivers MUST NOT contain unbounded loops

- **Determinism Requirement**: Avoid arbitrary wait loops. Clearly define maximum blocking times.
- **Polling vs. Interrupts**: Avoid tight polling loops. Use hardware interrupts (`InputPin::wait_for_low()` or `Exti`) to yield execution back to the executor to guarantee throughput and minimize system latency.

## 7. Strict API Safety Boundaries
Types protect state, but functions should protect behavior. Always enforce runtime and compile-time boundaries.
- **Validate Everything**: Do not assume valid input from user-space. Return explicit Results.
  ```rust
  // DO NOT DO THIS
  pub fn set_timing_budget(ms: u16);
  
  // DO THIS
  pub fn set_timing_budget(ms: u16) -> Result<(), Error<SPI::Error>>;
  ```
- **Type-Safe Configurations**: Prefer builders and typed configurations to prevent impossible configurations at compile time.

## 8. System-Level Integration Guidance
A crate is useless if it is hard to integrate into an overarching factory control plane. Documentation should move beyond "how to call this function" to "how this fits into a system."
- **Examples Folder**: Include robust examples of system-level integration.
- **Integration Patterns**: Document how the driver integrates with event-driven loops, how it binds to an async executor (like `embassy`), and what a typical control loop architecture looks like for the component.


## 9. Versioning & Stability

- Follow Semantic Versioning (SemVer)
- Breaking changes MUST:
  - increment major version
  - be documented clearly
- Public APIs must remain stable across minor versions