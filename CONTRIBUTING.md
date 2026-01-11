# Contributing to OpenNeato

First off, thanks for taking the time to contribute! 🎉

## Development Workflow

1.  **Fork the repository** on GitHub.
2.  **Clone your fork** locally: `git clone https://github.com/YOUR-USERNAME/OpenNeato.git`
3.  **Create a branch** for your feature: `git checkout -b feature/amazing-feature`
4.  **Make your changes**. Please follow the existing code style (Python PEP8).
5.  **Test your changes** on hardware if possible, or simulate.
6.  **Commit your changes** with meaningful messages.
7.  **Push to the branch**: `git push origin feature/amazing-feature`
8.  **Open a Pull Request**.

## Project Structure
* `firmware/`: ROS 2 packages (`driver`, `nav`, `interfaces`).
* `web_interface/`: FastAPI backend and HTML/JS frontend.
* `installer/`: Deployment scripts.

## Hardware Setup
Please refer to `README.md` for the required hardware (Radxa Zero 3W, Buck Converter, etc.).
