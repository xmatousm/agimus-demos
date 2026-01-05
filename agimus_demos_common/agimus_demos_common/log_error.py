"""Module for the LogError action."""

from typing import List

import launch.logging

from launch.action import Action
from launch.frontend import Entity
from launch.frontend import expose_action
from launch.frontend import Parser  # noqa: F401
from launch.launch_context import LaunchContext
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.substitution import Substitution
from launch.utilities import normalize_to_list_of_substitutions


@expose_action('log_error')
class LogError(Action):
    """Action that logs a message when executed."""

    def __init__(self, *, msg: SomeSubstitutionsType, **kwargs):
        """Create a LogInfo action."""
        super().__init__(**kwargs)

        self.__msg = normalize_to_list_of_substitutions(msg)
        self.__logger = launch.logging.get_logger('launch.user')

    @classmethod
    def parse(
        cls,
        entity: Entity,
        parser: 'Parser'
    ):
        """Parse `log` tag."""
        _, kwargs = super().parse(entity, parser)
        kwargs['msg'] = parser.parse_substitution(entity.get_attr('message'))
        return cls, kwargs

    @property
    def msg(self) -> List[Substitution]:
        """Getter for self.__msg."""
        return self.__msg

    def execute(self, context: LaunchContext) -> None:
        """Execute the action."""
        self.__logger.error(
            ''.join([context.perform_substitution(sub) for sub in self.msg])
        )
        return None
