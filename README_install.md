### install
sudo apt-get install libglfw3-dev libglm-dev
sudo dpkg -i biulab_2.14.8+Ubuntu2004_amd64_Release
### configure
```
copy plugin/simplesoft/config.json  bin/mujoco_plugin/
copy libsimplesoft.so bin/mujoco_plugin/
```
### run
./bin/simulate model/plugin/simplesoft/soft_test_model.xml

### debug
gdb --args bin/simulate model/plugin/simplesoft/soft_test_model.xml 

