package org.firstinspires.ftc.teamcode.Functions;


//Interface used for embedding autonomous programs within each other
public interface autoBase {
    //called by handler program on first initialization
    //gives out the basic info needed for the program to run
    public void init(int Alliance, int Auto, int Position, int EndPosition);
    //Loops that is run between initialization and the main loop
    //Gets called again whenever the program reaches a callback point
    public void init_loop();
    //Initialization of main loop. Ran once before going to main loop.
    public void loop_init();
    //Main loop, used to handle movement code, etc.
    public void loop();
    //Ran once when the program is closed. Used to save or terminate things.
    public void end();
}
