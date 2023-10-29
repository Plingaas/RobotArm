#ifndef PROGRAM_HPP
#define PROGRAM_HPP

#include "../math.hpp"
#include <vector>

class Program {

public:

    float r = 0.7f;

    /**
     * @brief Holds the program for the robot arm.
     *
     * Each Vector4 represent a position and how long the robot should
     * use to reach that position. The r variable is a multiplier that
     * is used to reduce the time it takes to reach so that the robot
     * moves faster. This is used since the game has an upgrade for robot
     * speed.
     *
     * The robot will linearly interpolate between positions.
     */
    std::vector<Vector4> program = {{0.0f,    -350.0f, 200.0f, 1.0f * r},
                                    {300.0f,  -100.0f, 146.0f, 1.0f * r},
                                    {300.0f,  -100.0f, 146.0f, 0.25f * r},
                                    {300.0f,  -100.0f, 96.0f,  0.5f * r},
                                    {300.0f,  -100.0f, 96.0f,  0.25f * r},
                                    {300.0f,  -100.0f, 146.0f, 1.0f * r},
                                    {300.0f,  -100.0f, 146.0f, 0.25f * r},
                                    {0.0f,    -350.0f, 200.0f, 1.0f * r},
                                    {-360.0f, -420.0f, -47.8f, 1.0f * r},
                                    {-360.0f, -420.0f, -47.8f, 0.25f * r},
                                    {-360.0f, -420.0f, -97.8f, 0.5f * r},
                                    {-360.0f, -420.0f, -97.8f, 0.25f * r},
                                    {-360.0f, -420.0f, -47.8f, 1.0f * r},
                                    {-360.0f, -420.0f, -47.8f, 0.25f * r}};

    // Linear interpolation variable 0 -> 1.
    float lerp = 0.0f;

    Vector3 position{0, 0, 0};
    unsigned int index = 0;
    unsigned int commands = program.size();
    bool repeat = false;
    bool running = false;

    // Which index of the program that the robot arm should pick an item up
    unsigned int pickIndex = 4;

    // Which index of the program that the robot arm should drop an item.
    unsigned int dropIndex = 11;

    // If an item is currently being held.
    bool isHolding = false;

    /**
     * @brief Generates a drop sequence for new drop positions
     *
     * Items are never placed on the same positions, therefore we
     * also need to modify the program for new drop positions. This
     * function replaces the current program's drop sequence by
     * generating a new sequence.
     *
     * When new robots are created the pickup position also changes. This
     * function will calculate both pickup and drop positions.
     *
     * @param pos The drop/pickup position.
     * @param drop A boolean describing if it is a drop position or pick up.
     * @param z_offset The offset for the robot arm before and after ir drops or picks up.
     *
     * @return void.
     */
    void generateSequence(Vector3 pos, bool drop, float z_offset = 75.0f);

    /**
     * @brief Sets the resting position of the robot.
     *
     * @param pos Sets the position.
     * @param w Sets the time to move to this position from another.
     *
     * @return void.
     */
    void setRestPosition(Vector3 pos, float w = 1.0f) {
        program[0] = {pos.x, pos.y, pos.z, w * r};
        program[7] = {pos.x, pos.y, pos.z, w * r};
    }

    /**
     * @brief Clears the program
     *
     * @return void.
     */
    void removeAll() { program = std::vector<Vector4>{}; };

    /**
     * @brief Updates the program
     *
     * This function updates everything related to the program
     * such as the lerp variable, the total time for the current move
     * and also moves on to the next command in the program if it should.
     *
     * @param dt The amount of time the last frame took.
     */
    void update(float dt);

    /**
     * @brief Sets the program to a new program.
     *
     * @param program_ The new program.
     *
     * @return void.
     */
    void setProgram(const std::vector<Vector4> &program_);

    /**
     * @brief Gets the current command in the program.
     *
     * @return Returns a Vector4 with the command.
     */
    Vector4 getCurrent() { return program.at(index); };

    /**
     * @brief Gets the next command in the program.
     *
     * @return Returns a Vector4 for the next command.
     */
    Vector4 getNext() {
        if (index == commands - 1) return program.front();
        else return program.at(index + 1);
    };

    /**
     * @brief Adds a pause at the end of the program.
     *
     * This is useful when creating new programs at it allows
     * you to add pauses at specific locations.
     *
     * @param time The amount of seconds to pause
     *
     * @return void.
     */
    void addPause(float time);

    /**
     * @brief Adds a new command to the program.
     *
     * If no n is specified the new command is added
     * to the back of the program.
     *
     * @param command The command to add.
     * @param n The index where the command should be added.
     *
     * @return void.
     */
    void add(Vector4 command, int n = -1);

    /**
     * @brief Removes a command.
     *
     * @param n The index to remove from the program.
     */
    void remove(int n);

    /**
     * @brief Sets the time multiplier variable.
     *
     * @param new_r The new time multiplier.
     *
     * @return void.
     */
    void setR(float new_r);

private:

    /**
     * @brief Extracts the position from a command.
     *
     * @param command The command to extract from.
     *
     * @return Returns the position as a Vector3.
     */
    Vector3 extractPosition(Vector4 command) { return {command.x, command.y, command.z}; };
};


#endif //ROBOTCONTROLLER_PROGRAM_HPP
