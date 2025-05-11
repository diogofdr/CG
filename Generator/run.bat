@echo off
:: Builds generator.exe with MSVC.  Requires VS 2022 or 2019.

:: 1) load compiler vars – change “Community” to “Professional” if needed
call "%ProgramFiles(x86)%\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"

:: 2) compile
cl /std:c++17 /EHsc /O2 /nologo Generator.cpp /Fe:generator.exe
