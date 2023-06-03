#include <iostream>

int main() {
	std::cout << "Hello C++\n";
	FILE* right_wheel = popen("rostopic echo /db4/wheels_driver_node/wheels_cmd/vel_right", "r");
	char right_wheel_line[100];

	while (fgets(right_wheel_line, sizeof right_wheel_line, right_wheel)) {
		std::cout << "Read from C++: " << right_wheel_line << "\n";
	}
	pclose(right_wheel);
	printf("Bye C++\n");
	return 0;
}

