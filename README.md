# PMW3389 Zephyr Driver Module

## Usage

This is a west module.
You can integrate it in your application by adding the project to your west manifest:

```yaml
manifest:
  remotes:
    - name: teamspatzenhirn
      url-base: https://github.com/teamspatzenhirn
  projects:
    - name: pmw3389_zephyr_driver
      remote: teamspatzenhirn
      revision: <current commit hash>
      path: modules/pmw3389
```

Enable the sensor by defining its location and properties in the devicetree (overlay).
The resolution is specified in counts per inch, and care should be taken to ensure that the
sensor-internal accumulator does not overflow (±32767).
The sensor supports a maximum SPI clock frequency of 2MHz.

```c
&lpspi4 {
    motion_sensor: pmw3389@a {
        compatible = "pixart,pmw3389";
        spi-max-frequency = <2000000>; // 2MHz
        resolution = <200>; // overflow in 0.5s at 8m/s -> 200 CPI
        status = "okay";
        reg = <10>;
    };
};
```

Example of reading sensor values:

```c++
#define MOTION_SENSOR_NODE DT_NODELABEL(motion_sensor)
constexpr const device *motion_sensor_dev = DEVICE_DT_GET(MOTION_SENSOR_NODE);

void sensor_readout() {
    sensor_sample_fetch(motion_sensor_dev);
    sensor_value v_x{};
    sensor_value v_y{};
    sensor_channel_get(motion_sensor_dev,
                       (sensor_channel) sensor_channel_pmw3389::SENSOR_CHAN_PMW3389_DISTANCE_X,
                       &v_x);
    sensor_channel_get(motion_sensor_dev,
                       (sensor_channel) sensor_channel_pmw3389::SENSOR_CHAN_PMW3389_DISTANCE_Y,
                       &v_y);
    LOG_INF("DELTA_X: %f, DELTA_Y: %f", sensor_value_to_double(&v_x), sensor_value_to_double(&v_y));
}
```

## Writing Out-Of-Tree Drivers
Since this was kind of a pain to figure out, here are a few helpful links if you want to do this:
* https://jdelaney.me/posts/zephyr-oot-modules/
* https://interrupt.memfault.com/blog/building-drivers-on-zephyr
* https://blog.golioth.io/adding-an-out-of-tree-sensor-driver-to-zephyr/
* https://github.com/zephyrproject-rtos/zephyr/tree/main/samples/application_development/out_of_tree_driver
