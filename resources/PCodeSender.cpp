// Compilation line:
// g++ -g -o PCodeSender PCodeSender.cpp

// =======================================================================================================
// serial communication handling script
// =======================================================================================================
// This script is made to simplify communication over serial with a kuka KRC1 robot controller.
// As a serial protocol, 3964R is used, since it seems to me to be the most useable one the KRC1 supports.
// For linux, just compile it with g++ with a -g flag, i.e.: g++ -g -o testExecutable serialDemo.cpp
// Since the first  bit in DIN66003 character isn't used, it is safe to use signed chars to store
// the bytes. This simplifies the covnersions from string to char and back.
// =======================================================================================================

// Standard libraries
#include <algorithm>
#include <ios>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <string>
#include <cstring>
#include <iterator>
#include <fstream>
#include <sstream>
#include <ctime>
#include <bits/stdc++.h>

// Linux headers
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

// general serial settings
#define MAX_READ_BUFFER_SIZE 256            // in bytes
#define DEFAULT_SERIAL_READ_TIMEOUT 10      // in deciseconds
#define DEFAULT_SERIAL_MIN_BYTES_TO_READ 0  // in bytes

// DIN 66003 character encoding shorthand for thsoe that are used directly in the 3964R protocol
// This is only added to improve readability
#define NUL 0x00
#define STX 0x02
#define ETX 0x03
#define DLE 0x10
#define NAK 0x15

bool setBaseSerialSettings(int serialPort) {
    // create new termios settings struct
    struct termios tty;

    // read in existing settings, handle potential error
    if(tcgetattr(serialPort, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        return false;
    }

    tty.c_cflag &= ~PARENB; // clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // clear all bits that set the data size
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // disable echo
    tty.c_lflag &= ~ECHOE; // disable erasure
    tty.c_lflag &= ~ECHONL; // disable new-line echo
    tty.c_lflag &= ~ISIG; // disable interpretation of INTR, QUIT and SUSP
    tty.c_lflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow control
    tty.c_lflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONCLR; // prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = DEFAULT_SERIAL_READ_TIMEOUT; // wait for up to 1s, returning as soon as any data is received
    tty.c_cc[VMIN] = DEFAULT_SERIAL_MIN_BYTES_TO_READ;

    // set in/out baud rate
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Save tty settings, also checking for error
    if (tcsetattr(serialPort, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return false;
    }
    return true;
}

char getBCCByte(std::vector<char> message, bool skipLast = true) {
    // resultant variable
    int xorArr = 0;

    // pop the alst character if needed
    if(skipLast) { message.pop_back(); }

    // iterating through every element in the array
    for (auto& byte : message) {
        xorArr = xorArr ^ byte;
    }

    // return the XOR
    return xorArr;
}

char getBCCByte(char[] message, int messageLength, bool skipLast = true) {
    // resultant variable
    int xorArr = 0;

    // pop the alst character if needed
    if(skipLast) { messageLength -= 1; }

    // iterating through every element in the array
    for (size_t i = 0; i < messageLength; i++) {
        xorArr = xorArr ^ message[i];
    }

    // return the XOR
    return xorArr;   
}

printBytesInHex(std::vector<char>& bytes) {
    for (auto&& byte : bytes) {
        std::cout << "0x";
        std::cout << std::hex << (0xFF & byte) << "\n";
    }
}

printBytesInHex(char bytes[], int amount) {
    for (size_t i = 0; i < amount; i++) {
        std::cout << "0x";
        std::cout << std::hex << (0xFF & bytes[i]) << "\n";
    }
}

bool doesByteOccurInInput(int serialPort, char byte) {
    char readBuf[MAX_READ_BUFFER_SIZE]{};
    int numBytes = read(serialPort, &readBuf, sizeof(readBuf));

    if(numBytes == 0) { return false; }

    if(readBuf[0] == byte) { return true; }
    else { return false; }
}

std::string readMessage(int serialPort, bool printBytes = false) {
    if(doesByteOccurInInput(serialPort, STX)) {
        // acknowledge the request to be sent amessage from the robot
        char acknowledgeByte[] = { DLE };
        write(serialPort, acknowledgeByte, sizeof(acknowledgeByte));

        char readBuf[MAX_READ_BUFFER_SIZE]{};
        std::vector<char> message{};
        bool endOfMessageReached = false;

        while(!endOfMessageReached) {
            // reset the buffer at the start of the iteration to NUL
            std::fill(std::begin(readBuf), std::end(readBuf), NUL );
            // check if new bytes have been received
            int numBytes = read(serialPort, &readBuf, sizeof(readBuf));

            for(size_t i = 0; i < numBytes; i++) {
                // if NAK is sent, stop the thingie
                if(readBuf[i] == NAK) { return std::string{"NAK"}; }

                // add te new byte to the total message buffer
                message.push_back(readBuf[i]);

                // check that this byte was not the last of the total message
                // this is signified by the following bytes in this order: DLE, ETX, BCC
                // BCC is a byte to check that no errors were puicked up during the transmission, thus it can not be used to detect the end of message
                size_t currentMessageLength = message.size();
                if(message[currentMessageLength - 2] == ETX && message[currentMessageLength - 3] == DLE) {
                    endOfMessageReached = true;
                }
            }
        }

        if(printBytes) { printBytesInHex(message); }

        // check that the BCC byte is correct, thus concluding that the message was received correctly
        if(getBCCByte(message, true) != message.back()) { return std::string{"NAK"}; }

        // since the message was received correctly, an acknowledgement if the end of message can be sent to the robot
        write(serialPort, acknowledgeByte, sizeof(acknowledgeByte));

        // remove the irrelevant characters from the end of message
        for (size_t i = 0; i < 3; i++) { message.pop_back(); }

        // convert the vector fo characters to an actual string for later processing
        return std::string{message.begin(), message.end()};
    } else {
        return std::string{"NUL"};
    }
}

bool sendMessage(int serialPort, std::string message) {
    char readBuf[MAX_READ_BUFFER_SIZE]{};

    // send a STX byte to ask the robot controller if it can accept a new message
    char askToSendMessageByte[] = { STX };
    write(serialPort, askToSendMessageByte, sizeof(askToSendMessageByte));

    // check if the robot controller responds with an acknowledgement
    if(!doesByteOccurInInput(serialPort, DLE)) { return false; }

    // conver the message to an array, ensuring that the bytes to be sent all lay in line with each other in memory
    char revisedMessage[message.size( + 3)]{};
    strcpy(revisedMessage, message.c_str());

    // set the bytes to signify the end of the message
    revisedMessage[message.size() + 0] = DLE:
    revisedMessage[message.size() + 1] = ETX:

    // calculate and update the last byte of the message to te BCC byte
    revisedMessage[message.size() + 2] = getBCCByte(revisedMessage, message.size() + 3, true);

    // send it over the serial connection
    write(serialPort, revisedMessage, sizeof(revisedMessage));

    // check if the robot controller acknowledges the end of message
    if(!doesByteOccurInInput(serialPort, DLE)) { return false; }

    return true;
}

std::string getMovementMessageFromAxisAngles(
    char type,
    int x, int y, int z, int a, int b, int c,
    int x2=0, int y2=0, int z2=0, int a2=0, int b2=0, int c2=0
) {
    std::string message(1, type);
    message += " ";

    message += std::to_string(x) + " ";
    message += std::to_string(y) + " ";
    message += std::to_string(z) + " ";
    message += std::to_string(a) + " ";
    message += std::to_string(b) + " ";
    message += std::to_string(c) + " ";

    message += std::to_string(x2) + " ";
    message += std::to_string(y2) + " ";
    message += std::to_string(z2) + " ";
    message += std::to_string(a2) + " ";
    message += std::to_string(b2) + " ";
    message += std::to_string(c2);

    return message;
}

void blockSendCommandUntillRequest(int& serialPort, int& iteration, std::string& instruction) {
    // check if the controller has requested a new command
    std::string readResult = "";
    while(readResult != std::string("#")) {
        readResult = readMessage(serialPort);
        std::cout << readResult << std::endl;
        // sleep(1);
    }

    // now that the controller has requested a command, send it
    iteration++;
    printf("sending message %i: ", iteration);
    printf(iteration.c_str());
    printf("\n");

    sendMessage(serialPort, instruction);
}

int main(void) {
    printf("Loop started!\n");

    // open the serial port
    int serialPort = open("/dev/ttyS0", 0_RDWR);

    // set seruial settings like baud rate, input/output process etc
    setBaseSerialSettings(serialPort);

    int iteration = 0;

    std::ifstream inputFile("KapBottom-Final-Pass-v2.txt");
    std::string inputLine;
    while (std::getline(inputFile, inputLine)) {

        // extract the coordinates from the file
        char moveType;
        int x, y, z, a, b, c;
        x = 0;
        y = 0;
        z = 0;
        a = 90;
        b = 0;
        c = 0;

        int x2, y2, z2, a2, b2, c2;
        x2 = 0;
        y2 = 0;
        z2 = 0;
        a2 = 0;
        b2 = 0;
        c2 = 0;

        std::stringstream iss(inputLine);

        // split up the line
        std::string tempString;
        std::vector<std::string> input;
        while(std::getline(iss, tempString, ' ')) {
            input.push_back(tempString);
        }

        moveType = input[0].front();
        x = (int)std::stoi(input[1]);
        y = (int)std::stoi(input[2]);
        z = (int)std::stoi(input[3]);
        if(moveType == 'C') {
            moveType == 'L';
            x2 = (int)std::stoi(input[4]);
            y2 = (int)std::stoi(input[5]);
            z2 = (int)std::stoi(input[6]); 
        }

        // convert the coordinates to the message
        std::string instruction = getMovementMessageFromAxisAngles(moveType, x, y, z, a, b, c, x2, y2, z2, a2, b2, c2);

        blockSendCommandUntillRequest(serialPort, iteration, instruction);
    }

    // after the full file has been read out, send the command that communicates end of file
    std::string instruction = getMovementMessageFromAxisAngles('E', 0, 0, 0, 0, 0, 0);
    blockSendCommandUntillRequest(serialPort, iteration, instruction);

    close(serialPort);
    return 0;
}