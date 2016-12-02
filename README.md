A detailed installation guide could be found [here](install.txt).

#  Installation guide
This project works with Ubuntu 14.04.

[Installation guide](install.txt)

## Issues
If you have **peer not authenticated** error, make sure that you are using Java 8.

If ARLocROS gives execption "no opencv-248 in java library path" you have to add the path to the native opencv library.
In linux that can be done, normally with:

    ```
    export  LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/jni/
    ```
 

	
