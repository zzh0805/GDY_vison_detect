# -*- coding: utf-8 -*-
"""统一日志工具：文件(按天滚动，UTF-8) + 控制台，含时间/级别/线程名。

用法：
    from vision_solver.logging_utils import setup_logging, get_logger
    setup_logging(PROJECT_ROOT, config)   # 服务启动时调用一次
    log = get_logger()
    log.info("...") / log.warning(...) / log.error(...)
"""
from __future__ import annotations

import logging
import logging.handlers
import time
from pathlib import Path
from typing import Any, Mapping, Optional

LOGGER_NAME = "vision_service"
_LOG_FORMAT = "[%(asctime)s] [%(levelname)s] [%(threadName)s] %(message)s"
_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"

_LEVELS = {
    "DEBUG": logging.DEBUG,
    "INFO": logging.INFO,
    "WARNING": logging.WARNING,
    "ERROR": logging.ERROR,
    "CRITICAL": logging.CRITICAL,
}


def get_logger(name: Optional[str] = None) -> logging.Logger:
    """获取服务日志器。未 setup 时使用 Python 默认根配置（不强制输出）。"""
    return logging.getLogger(name or LOGGER_NAME)


def cleanup_expired_logs(log_file: Path, retention_days: int) -> None:
    """服务启动时主动检查并删除超过 retention_days 的按天滚动日志。

    按文件修改时间判断（不依赖文件名格式），能覆盖历史遗留的任意旧日志；
    当前正在写入的 service.log（无日期后缀）不会被匹配删除。
    """
    if retention_days < 1:
        return
    prefix = log_file.name + "."
    cutoff = time.time() - retention_days * 86400.0
    removed = 0
    try:
        for candidate in log_file.parent.glob(prefix + "*"):
            try:
                if candidate.is_file() and candidate.stat().st_mtime < cutoff:
                    candidate.unlink()
                    removed += 1
            except OSError:
                continue
    except OSError:
        return
    if removed:
        get_logger().info(
            "日志启动清理: 删除 %d 个超过 %d 天的旧日志文件",
            removed, retention_days)


def setup_logging(root: Path,
                  config: Optional[Mapping[str, Any]] = None) -> None:
    """根据 workflow.yaml 的 system.log 段配置服务日志。

    config 字段：
      enabled:       bool   总开关（默认 True）
      level:         str    DEBUG/INFO/WARNING/ERROR（默认 INFO）
      file:          str    日志文件路径（相对 root，默认 service.log）
      retention_days:int    按天滚动并保留最近 N 天（默认 7）；
                          每次服务启动时主动扫描并清理超过 N 天的旧日志
      console:       bool   同时输出到控制台（默认 True）
    """
    cfg = dict(config or {})
    if not bool(cfg.get("enabled", True)):
        return
    level_name = str(cfg.get("level", "INFO")).upper()
    level = _LEVELS.get(level_name, logging.INFO)
    log_file = Path(str(cfg.get("file") or "service.log")).expanduser()
    if not log_file.is_absolute():
        log_file = (root / log_file).resolve()
    retention_days = max(1, int(cfg.get("retention_days", 7)))
    console = bool(cfg.get("console", True))

    logger = logging.getLogger(LOGGER_NAME)
    logger.setLevel(level)
    logger.propagate = False
    for handler in list(logger.handlers):
        logger.removeHandler(handler)
        handler.close()

    formatter = logging.Formatter(_LOG_FORMAT, datefmt=_DATE_FORMAT)
    log_file.parent.mkdir(parents=True, exist_ok=True)
    # 每次服务启动先主动清理超过 retention_days 的旧日志（开机自启后立即整理）。
    cleanup_expired_logs(log_file, retention_days)
    # 每天零点滚动出一个 service.log.YYYY-MM-DD 文件，保留最近 N 天。
    file_handler = logging.handlers.TimedRotatingFileHandler(
        log_file, when="midnight", backupCount=retention_days,
        encoding="utf-8")
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    if console:
        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)
    logger.info("日志已初始化: level=%s file=%s 保留最近%d天（启动时自动清理）",
                level_name, log_file, retention_days)


def log_exception(logger: logging.Logger, message: str) -> None:
    """打印带完整 traceback 的错误日志（logger.exception 的封装）。"""
    logger.error(message, exc_info=True)
