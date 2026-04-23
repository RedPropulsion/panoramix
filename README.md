# Panoramix

_The druid_

The project is based on [Zephyr](https://www.zephyrproject.org/).

## Structure

- (`boards`)[./boards/] contains the board definition with `.dts` files.
- (`apps`)(./apps/) contains applications' code (in this case just for the `AsterICS`).

## Getting started (setup)

[Zephyr](https://www.zephyrproject.org/) has the concept of workspace which wraps projects. This can be used to setup multiple projects with just one workspace to reduce disk usage.

If you have in-depth knowledge of Zephyr, feel free to integrate this project with your existing workspace. Otherwise stick to these steps.

(_Disclaimer_: there are more information and more way to setup the environment at [the official guide](). This is just what I think may be best).

This setup uses a [Python virtual environment](https://docs.python.org/3/library/venv.html) to handle dependencies.

### venv setup
```bash
mkdir panoramix-workspace # Or whatever name you like for your workspace
cd panoramix-workspace

python -m venv .venv --prompt zephyr
source .venv/bin/activate # This will add (zephyr) to your prompt
```

_(Remember to `deactivate` when you're not developing this project!)_

### Workspace setup

_(Inside the virtual environment)_
```bash
pip install west

west init -m git@github.com:RedPropulsion/panoramix
```

Inside your workspace you'll see a `panoramix` folder, containing the application code from the repo.

```bash
west update # Fetches all depencendies
```

This may take a while! So seat back and relax.

Once it is done, install Zephyr tools

```bash
west zephyr-export
west packages pip --install
west sdk install --toolchain arm-zephyr-eabi
```

## Usage

Once everything is setup, you'll develop off the `panoramix` folder. Remember to activate the environment to access the `west` tool.

You'll need to run `west update` everytime `west.yml` is updated.

- `west build apps/asterics` builds the AsterICS code.
- `west flash` flashes it with the STLink. It uses [`openocd`](https://openocd.org/), so make sure to have it installed.
