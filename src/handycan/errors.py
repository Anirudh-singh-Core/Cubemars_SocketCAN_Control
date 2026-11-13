"""Actionable errors; errors never trigger automatic reconnect or motor resume."""
class CANError(RuntimeError): pass
class OwnershipError(CANError): pass
class InterfaceError(CANError): pass
class FeedbackError(CANError): pass
class SessionFault(CANError): pass
class BusyError(CANError): pass
