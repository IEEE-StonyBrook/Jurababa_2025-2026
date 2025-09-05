# Authors:
### Made with ❤️ by the Micromouse team!

## Awards:
- NYIT #2 Place Micromouse Classic.
- NYIT #1 and #3 Best Presentation.
- NYIT $250 Scholarship for Micromouse.
Isabella

## Codebase Standards
### 1. Use floats NOT doubles/longs (e.g. 0.0f, 2.0f).
We want to preserve memory even though we have a lot on the Pico.

### 2. Use object-oriented standards for creating new classes.
Notice how Drivetrain is built from Encoders, IMUs, Motors, and ToFs. You can go through each of these objects and understand functions CONFINED to the object itself. Makes the codebase easier to understand for everyone.

### 3. Use C++ docs for all methods you write to explain your code to your peers.
Look at the existing code and the decoration for arguments and return types and emulate that.

### 4. Make a .h file and .cpp file for each new class
.h for variable and method declarations. .cpp for actual method/class writing.

### 5. Try to keep most of your includes in the .cpp file.
Makes code more efficient as we include .h files in most objects.

### 6. Add your new classes to the CMakeList.txt target directories each time you write a new class.

### 7. DRY - Don't repeat yourself!

### 8. Install clang-format extension on your IDE and use the Google formatting (don't change it)
We need to stay consistent in our formatting for the entire project! Googlers made a great one the codebase currently uses. To format on VSCode, right click and press "Format Document With."
