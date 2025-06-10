from typing import TypeVar, Generic, Type, Optional
from abc import abstractmethod

_T = TypeVar("_T")


# class foo(Generic[_T]):
#     def __init__(self, T: type):
#         self._T = T

#     def make_obj(self) -> _T:
#         return self._T()


# class processing(Generic[_T]):

#     def __init__(self, cls: type):
#         assert isinstance(_T, cls)
#         self.T = cls

#     @staticmethod
#     def make(cls) -> foo[_T]:
#         return foo[_T](cls)


# a = processing[int].make(int).make_obj()
# print(a)


class GenericProcessor(Generic[_T]):

    _U = TypeVar("_U", bound="GenericProcessor")

    def __set_T(self, c: Type[_T]):
        """
        Set an interval variable to save the class
        """
        self._T = c

    def make_T(self, **kwargs) -> _T:
        """
        Initialize a new
        """
        return self._T(**kwargs)

    @staticmethod
    def init(p: Type[_U], c: Type[_T]) -> _U:
        o = p()
        o.__set_T(c)
        return o


class processing(GenericProcessor[_T]):

    def foo(self) -> _T:
        return self._T()


def make_processor(c: Type[_T]) -> processing[_T]:
    return GenericProcessor.init(processing, c)


def init_val(c: Type[_T]) -> _T:
    return c()


c = make_processor(int)
print(c)
o = c.foo()
print(o)

a = init_val(int)
print(a)
