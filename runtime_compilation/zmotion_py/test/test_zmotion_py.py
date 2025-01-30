from runtime_compilation.zmotion_py import (
    Handle_Wrapper,
    zmotion_close,
    zmotion_direct,
    zmotion_execute,
    zmotion_init,
    zmotion_open_ethernet,
)


def test_the_zmotion_py_work() -> None:
    """Test whether the zmotion_py works."""
    assert zmotion_init is not None
    assert zmotion_open_ethernet is not None
    assert zmotion_execute is not None
    assert zmotion_direct is not None
    assert zmotion_close is not None
    assert Handle_Wrapper is not None
