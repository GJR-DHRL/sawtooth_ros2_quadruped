// #include "sawtooth_controller.h" // include controller header file 

// int main(int argc, char** argv) {  
//     rclcpp::init(argc,argv);    // Initializes the ROS2 system.
//     /*
//     -> argc and argv are the command-line arguments passed to your program.
//     -> argc = number of arguments.
//     -> argv = actual argument values (as an array of C-style strings).
//     -> ROS2 requires these because it often accepts command-line arguments (e.g., remapping topics, logging levels).

//     -> Command line arguments are values or parameters passed to a program when it is executed from the command line or terminal. They allow users to customize the program's behavior or provide input during runtime without modifying the source code. 
//     */
//     rclcpp::spin(std::make_shared<SawtoothController>()); 
//     /*
//     -> Creates an instance of your node class SawtoothController using std::make_shared (which manages memory automatically with a smart pointer).
//     -> rclcpp::spin(node) starts an event loop that keeps your node alive.
//     -> It continuously listens for callbacks (like subscriber messages, timers, or service calls).
//     -> Without spin(), your program would just exit immediately after creating the node.
//     -> Essentially: this is the "heartbeat" of your ROS2 node.
//     */
//     rclcpp::shutdown();         // Ctrl + c
//     /*
//     -> Shuts down the ROS2 system.
//     -> Cleans up allocated resources, closes publishers/subscribers, and ensures everything terminates gracefully.
//     */
//     return 0;
// }





