// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Based on Drew Noakes 2013-2016

#include "joystick.hh"
#include <unistd.h>
#include <chrono>
#include <cmath>
using namespace std;
using namespace std::chrono;

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))


int main(int argc, char** argv)
{
  // Create an instance of Joystick
  Joystick joystick("/dev/input/js1");

  // Ensure that it was found and that we can use it
  if (!joystick.isFound())
  {
    printf("open failed.\n");
    exit(1);
  }

  int last_left=0, last_right=0, uncapped=0, bin_button=1, cam_button=1, coll_button=1 ;
  auto start = high_resolution_clock::now();
  while (true)
  {
    // Restrict rate
    usleep(1000);

    // Attempt to sample an event from the joystick
    JoystickEvent event;
    if (joystick.sample(&event))
    {
      if (event.isButton())
      {
        //printf("Button %u is %s\n",
          //event.number,
          //event.value == 0 ? "up" : "down");
        //A=0;B=1;X=2;Y=3;
        if(event.number==5)
          uncapped=event.value!=0?1:0;
        if(event.number==0)//A
          bin_button=event.value!=0?0:1;
        if(event.number==1)//B
          bin_button=event.value!=0?2:1;
        if(event.number==2)//X
          coll_button=event.value!=0?0:1;
        if(event.number==3)//Y
          coll_button=event.value!=0?2:1;
        if(event.number==6)//Back
          cam_button=event.value!=0?0:1;
        if(event.number==7)//Start
          cam_button=event.value!=0?2:1;
      }
      else if (event.isAxis())
      {
        //printf("Axis %u is at position %d\n", event.number, event.value);
        int abb = (event.value>0?1:-1)*max(0,(event.value>0?event.value:-event.value)-100*64);
		if(event.number==1)
			last_left=max(-1000.,min(1000.,-abb/(uncapped?25:50)));
		else if(event.number==4)
			last_right=max(-1000.,min(1000.,-abb/(uncapped?25:50)));
      }
    }
    auto curr = high_resolution_clock::now();
    auto diff = curr-start;

    if(diff>=milliseconds(50)){
       //printf("!G 1 %d_!G 2 %d_\n",last_left,-last_right);
       printf("!%d,%d,%d,%d,%d\n",last_left,-last_right,bin_button, cam_button, coll_button);
       fflush(stdout);
       start= curr;
    }
  }
}
