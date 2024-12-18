# Simple GPS

A not-so-good but sorta-good-enough library for using UBLOX GPS over serial.



To build:

```
git submodule update --init --recursive
cd libnmea
make -j$(nproc)
sudo make install
cd ..
```

After libnmea is installed, this is a header-only library. Just make sure you link against libnmea when building downstream projects.

Note that `gps_buffer.h` reads the path of the device from the environment variable `UBLOX_PATH`, but defaults in to `/dev/ttyS1`. It also hardcodes the baud rate as 115200.

To use the code, for example:

```
#include <simple_gps/gps_buffer.h>

static double gnss_R_ref[9];
static double gnss_p_ref[3];

static void __gnss_init(void)
{

  // Launch the background thread to read the GPS.
  if(create_gps_thread() < 0)
    exit(1);

  printf("Starting GNSS Initialization\n");
  raw_gnss_t data;
  int count = 0;

  // Discard the first few messages.
  while(count < 10) {
    if(read_gps_buffer(&data) < 0)
    	continue;
    count += 1;
  }

  compute_reference(data.latitude_degrees,
                    data.longitude_degrees,
                    data.altitude,
                    gnss_R_ref,
                    gnss_p_ref);

  printf("GNSS Initialized\n");
}

static void __gnss_march(void)
{
  double result[3];
  raw_gnss_t new_data;
  if(read_gps_buffer(&new_data) >= 0 
     && convert_data_to_ned(new_data, gnss_R_ref, gnss_p_ref,result))
  {
    state_estimate.gnss[0] = (float)result[0];
    state_estimate.gnss[1] = (float)result[1];
    state_estimate.gnss[2] = (float)result[2];

    state_estimate.gnss_latlonalt[0] = new_data.latitude_degrees;
    state_estimate.gnss_latlonalt[1] = new_data.longitude_degrees;
    state_estimate.gnss_latlonalt[2] = new_data.altitude;

    state_estimate.gnss_time = new_data.time;
  }
}

static void __gnss_cleanup(void)
{
  stop_gps_thread();
}
```