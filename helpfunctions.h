#define TORQUE 1
#define NORMAL 2
#define SPEED 6

int arrlen(int *arr);
int cycle_num(int num, int dist, int max);

int arrlen(int *arr) {
	// Count number of entries in array
	int i = 0;
	while (arr[i] != '\0') {
		i++;
	}
	return i;
}

int cycle_num(int num, int dist, int max) {
  num += dist;
  if(dist > 0 && num > max) num = 0;
  if(dist < 0 && num < 0) num = max;
  return num;
}