"""
PENG Yuhang
events.py
轻量级事件系统
"""

from typing import Callable, List, Dict, Any
import logging

class EventManager:
    """
    简单的事件管理器
    支持多个事件名称，每个事件可以有多个回调函数
    """
    def __init__(self):
        # 事件名 -> 回调函数列表
        self._events: Dict[str, List[Callable]] = {}

    def register(self, event_name: str, callback: Callable) -> Callable:
        """
        注册事件回调（装饰器或直接调用均可）

        示例：
            @event_manager.on("task_received")
            def handle_task(task: str):
                ...

        或：
            event_manager.on("task_received", handle_task)
        """
        if event_name not in self._events:
            self._events[event_name] = []

        if callback not in self._events[event_name]:
            self._events[event_name].append(callback)

        # 返回自身，便于装饰器使用
        return callback

    def unregister(self, event_name: str, callback: Callable = None) -> None:
        """
        移除事件回调
        如果不传 callback，则移除该事件所有回调
        """
        if event_name not in self._events:
            return

        if callback is None:
            del self._events[event_name]
        else:
            self._events[event_name] = [
                cb for cb in self._events[event_name] if cb != callback
            ]
            if not self._events[event_name]:
                del self._events[event_name]

    def emit(self, event_name: str, *args, **kwargs) -> None:
        """
        触发事件，调用所有注册的回调函数
        """
        if event_name not in self._events:
            return

        callbacks = self._events[event_name].copy()  # 避免运行时修改列表
        for callback in callbacks:
            try:
                callback(*args, **kwargs)
            except Exception as e:
                logging.getLogger(__name__).error(
                    f"Error in callback for event '{event_name}': {e}"
                )

    def clear(self) -> None:
        """清空所有事件"""
        self._events.clear()

    def has_listeners(self, event_name: str) -> bool:
        """检查是否有监听者"""
        return event_name in self._events and bool(self._events[event_name])


event_manager = EventManager()

def register(event_name: str):
    """装饰器方式注册事件"""
    def decorator(callback: Callable):
        event_manager.register(event_name, callback)
        return callback
    return decorator

def unregister(event_name: str, callback: Callable = None):
    event_manager.unregister(event_name, callback)


def emit(event_name: str, *args, **kwargs):
    event_manager.emit(event_name, *args, **kwargs)