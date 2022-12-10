## 5.1 Test on Real Robot (UR10)
Here we provide the steps to test our method on a real robot.

**Robot control**

Robot is controlled via [this python software](https://github.com/SintefManufacturing/python-urx).

**Camera setup**

To deploy RealSense L515 camera,
Download and install the [librealsense SDK 2.0](https://github.com/IntelRealSense/librealsense)

**Start testing**

Then run the following code to start testing:
```
python test_in_real.py
```

if use ft:
1. self.use_ft = True, self.hardcode = False 
2. python rl_train.py --play-only --model=with_ft_final.zip

if without ft:
1. self.use_ft = False, self.hardcode = False
2. python rl_train.py --play-only --model=noft.zip

if flex-flip:
1. self.use_ft = False, self.flex= True
2. python rl_train.py --play-only

## Maintenance 
For any technical issues, please contact: Chao Zhao (czhaobb@connect.ust.hk).
