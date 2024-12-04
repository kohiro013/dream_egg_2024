$elf_file = Get-ChildItem -Path .\Debug -Recurse | Where-Object{$_.Name -like "*.elf"} | Select-Object fullname
STM32_Programmer_CLI -c port=SWD -w  $($elf_file.FUllName) -v -s