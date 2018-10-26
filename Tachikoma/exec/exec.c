#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

int main(int argc, char **argv)
{
	chdir("/home/lightning/personal-repo/robotbattle");

	//FD 3 -> 1000
	open("key",O_RDONLY);
	dup2(3, 1000);

	//FD 4 -> 1001
	open("server_robot.prg", O_RDONLY);
	dup2(4, 1001);

	//rewrite FD 0
        //close(0);
        //open("rbdata", O_RDONLY);

	char *NewArgv[] = {"Tachikoma", 0};
	execv("./Tachikoma", NewArgv);
}
