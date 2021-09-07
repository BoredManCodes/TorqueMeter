# cmake_minimum_required(VERSION <specify CMake version here>)
project(TorqueMeter)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(TorqueMeter
        "COSMOS Config Files/CAR_TEL/cmd_tlm/cmd.txt"
        "COSMOS Config Files/CAR_TEL/cmd_tlm/tlm.txt"
        "COSMOS Config Files/CAR_TEL/screens/accelerometer.txt"
        "COSMOS Config Files/CAR_TEL/screens/test_canvas.txt"
        "COSMOS Config Files/CAR_TEL/cmd_tlm_server.txt"
        Receiver.ino
        Transmitter.ino)
