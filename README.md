# sawyer_ik

I'm using uv to manage python environments and versions: https://docs.astral.sh/uv/
Install it here and follow the commands below.

```
uv venv venv-sawyer --python 3.10
source venv-sawyer/bin/activate
uv pip install -r requirements.txt
```

Now to run the code
```
python3 run_ik_test.py
```

This will generate a static html file that you can also drag and drop into any browser to visualize the robot.