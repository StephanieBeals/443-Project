#include <iostream>
#include <utility> // for std::pair

// Function to get an argument and return two elements
std::pair<int, int> getTwoElements(int argument) {
    int result1, result2;
    // Perform some computation or logic
    // For example, splitting the argument into two parts
    result1 = argument / 2;
    result2 = argument - result1;
    // Return the two elements as a pair
    return std::make_pair(result1, result2);
}

int main() {
    int argument = 10;
    
    // Call the function and receive the result as a pair
    std::pair<int, int> result = getTwoElements(argument);

    // Access and print the individual elements
    std::cout << "Result 1: " << result.first << std::endl;
    std::cout << "Result 2: " << result.second << std::endl;

    return 0;
}
