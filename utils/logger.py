import logging

logging.basicConfig(
    level = logging.INFO,
    format = "%(filename)s -> %(asctime)s-[%(levelname)s]: %(lineno)d - \n %(message)s",
    datefmt = "%H:%M:%S"
)

logger = logging.getLogger(__name__)