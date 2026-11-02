"""Operator-owned session configuration. No values are accepted from agent code."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
import re

AGENT_EFFORTS = ('high', 'xhigh', 'max')
PROVIDERS = ('claude-login', 'openrouter', 'anthropic')
PROFILES = ('rgbd', 'rgb-only')
DEMONSTRATION_KINDS = ('terminal_state', 'completion_sequence')
ENVIRONMENTS = ('real', 'sim')
IMAGE_SUFFIXES = ('.png', '.jpg', '.jpeg')


@dataclass(frozen=True)
class Configuration:
    root: Path                      # session directory under the artifact root
    task: str                       # short task id, e.g. "stack_cups"
    task_description: str = ''      # TASK.md body written by the operator
    robot: str = 'rby1'             # services.backends registry key
    # Backend-specific operator settings (bridge URLs, cameras, workspace box, ...).
    backend_options: dict = field(default_factory=dict)
    # Interpreter for the agent's own scripts (a venv without robot SDKs); None: the launcher's.
    agent_python: str | None = None
    observation_profile: str = 'rgbd'
    agent_provider: str = 'claude-login'
    agent_effort: str = 'xhigh'
    agent_model: str | None = None  # Provider default (Opus 5.5) when unset.
    session_seconds: int = 4 * 3600
    max_model_calls: int = 4000
    subagents: int = 0              # 0: the Agent tool is removed.
    # Operator-supplied images of the task (none by default): copied in this order into the
    # workspace's runtime/demonstrations/ and read by the in-context-action-learning skill.
    demonstration_images: list = field(default_factory=list)
    demonstration_kind: str = 'terminal_state'   # what the images show, see DEMONSTRATION_KINDS
    # An earlier session's agent-workspace (or any directory with memory/, handoff/, code/) whose
    # experience this session starts from: its memory and handoff are copied into the new workspace.
    experience_from: str | None = None
    # Where the robot is: the physical robot or a simulation of it. Experience carries this as its source, so a
    # real session can tell sim-measured numbers from its own (see the sim-to-real skill).
    environment: str = 'real'

    def __post_init__(self):
        object.__setattr__(self, 'root', Path(self.root).resolve())
        from services.storage_root import ENVIRONMENT, artifact_root
        if not self.root.is_relative_to(artifact_root()):
            raise ValueError(f'Session artifacts must be under {artifact_root()} (set {ENVIRONMENT} to change it)')
        if not re.fullmatch(r'[A-Za-z0-9_-]+', self.task):
            raise ValueError('Invalid task id')
        if not isinstance(self.task_description, str):
            raise ValueError('task_description must be text')
        from services.backends import BACKENDS
        if self.robot not in BACKENDS:
            raise ValueError(f"Unknown robot; choose one of {', '.join(sorted(BACKENDS))}")
        if not isinstance(self.backend_options, dict):
            raise ValueError('backend_options must be an object')
        if self.agent_python is not None and not (isinstance(self.agent_python, str)
                                                  and Path(self.agent_python).is_absolute()):
            raise ValueError('agent_python must be an absolute path to a python executable')
        if self.observation_profile not in PROFILES:
            raise ValueError(f'observation_profile must be one of {PROFILES}')
        if self.agent_provider not in PROVIDERS:
            raise ValueError(f'agent_provider must be one of {PROVIDERS}')
        if self.agent_effort not in AGENT_EFFORTS:
            raise ValueError(f'agent_effort must be one of {AGENT_EFFORTS}')
        if type(self.session_seconds) is not int or not 60 <= self.session_seconds <= 7 * 24 * 3600:
            raise ValueError('session_seconds must be 60..604800')
        if type(self.max_model_calls) is not int or self.max_model_calls < 1:
            raise ValueError('max_model_calls must be positive')
        if type(self.subagents) is not int or not 0 <= self.subagents <= 8:
            raise ValueError('subagents must be 0..8')
        if not isinstance(self.demonstration_images, list) or not all(
                isinstance(p, str) and Path(p).is_absolute() and Path(p).suffix.lower() in IMAGE_SUFFIXES
                for p in self.demonstration_images):
            raise ValueError(f'demonstration_images must be absolute paths to {IMAGE_SUFFIXES} files')
        if self.demonstration_kind not in DEMONSTRATION_KINDS:
            raise ValueError(f'demonstration_kind must be one of {DEMONSTRATION_KINDS}')
        if self.experience_from is not None and not (isinstance(self.experience_from, str)
                                                     and Path(self.experience_from).is_absolute()
                                                     and Path(self.experience_from).is_dir()):
            raise ValueError('experience_from must be an absolute path to an existing directory')
        if self.environment not in ENVIRONMENTS:
            raise ValueError(f'environment must be one of {ENVIRONMENTS}')

    @property
    def camera_geometry(self):
        return self.observation_profile == 'rgbd'

    @classmethod
    def from_dict(cls, value):
        return cls(**dict(value))

    @classmethod
    def load(cls, path):
        return cls.from_dict(json.loads(Path(path).read_text()))

    def to_dict(self):
        return json.loads(json.dumps(asdict(self), default=str))
