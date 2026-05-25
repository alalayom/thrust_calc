try:
    from .app import run
except ImportError:
    from app import run


if __name__ == "__main__":
    run()
