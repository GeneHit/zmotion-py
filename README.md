# zmotion_py
Python bindings of the ZMotion control card library

Zmotion office: http://www.zmotionglobal.com/

## Difference Between Manual Compilation and Runtime Compilation

### Manual Compilation
Manual compilation is the process where the source code is compiled into an 
executable binary before it is run. This is typically done using a compiler, 
which translates the high-level code into machine code that the computer can 
execute. The key characteristics of manual compilation are:
- **Pre-execution**: The code is compiled before execution.
- **Performance**: Generally results in faster execution since the code is 
already translated into machine language.
- **Error Checking**: Compilation errors are caught before the program runs.
- **Portability**: The compiled binary is specific to the architecture it was compiled for.

### Runtime Compilation
Runtime compilation, also known as Just-In-Time (JIT) compilation, occurs when 
the source code is compiled during the execution of the program. This is often 
used in environments like Java and .NET. The key characteristics of runtime 
compilation are:
- **On-the-fly**: The code is compiled during execution.
- **Flexibility**: Allows for optimizations based on the current execution context.
- **Performance**: May introduce overhead due to the compilation process happening at runtime.
- **Error Handling**: Errors may occur during execution, which can be harder to debug.

Both methods have their own use cases and advantages depending on the 
requirements of the application.