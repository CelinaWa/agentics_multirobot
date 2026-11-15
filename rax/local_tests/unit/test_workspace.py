"""Workspace generation for the real-robot agent: template, skills, blocks and contract."""
import json
from pathlib import Path
import re

import pytest

from services.workspace import ROBOT_FACTS, Setup, build, recipe_for, render
from services.workspace.templates import registry

ROOT = Path(__file__).resolve().parents[2]


METHOD_SKILLS = {'evidence-and-regression', 'prioritize-by-funnel', 'change-discipline', 'engineer-the-controller',
                 'spend-the-budget', 'learn-from-incidents', 'verify-committal-actions', 'measure-success',
                 'match-precision-to-error', 'avoid-overfitting'}


@pytest.mark.parametrize('profile', ['rgbd', 'rgb-only'])
def test_rby1_workspace_renders_its_own_robot_skills(tmp_path, profile):
    workspace = tmp_path/'ws'
    manifest = build(recipe_for(), Setup('rby1', 'tidy_table', 'claude', profile), workspace)
    for path in workspace.rglob('*.md'):
        text = path.read_text()
        assert '{{' not in text and '{%' not in text, path
        assert not re.search(r'[Rr]obo[Dd]ojo|BEHAVIOR|rehears|formal batch|container|[Ff]ranka|[Dd]exmate|Vega', text), path
    skills = set(p.name for p in (workspace/'.agents/skills').iterdir())
    assert {'manual-robot-control', 'experience-memory', 'live-stream', 'gemini', 'robot-interface',
            'whole-body-control', 'base-navigation', 'grasping', 'object-localization',
            'transfer-handoff', *METHOD_SKILLS} <= skills
    assert 'motion-planning' not in skills  # no planner is wired into this backend
    assert 'in-context-action-learning' not in skills  # no demonstration images in this session
    localization = (workspace/'.agents/skills/object-localization/SKILL.md').read_text()
    assert ('cannot back-project' in localization) == (profile == 'rgb-only')
    assert ('triangulat' in localization) == (profile == 'rgb-only')
    assert (workspace/'api/stream.py').is_file() and (workspace/'api/runtime.py').is_file()
    assert 'rby1_stream' in (workspace/'.agents/skills/live-stream/SKILL.md').read_text()
    assert 'real hardware' in (workspace/'MCP_SESSION.md').read_text()
    interface = (workspace/'.agents/skills/robot-interface/SKILL.md').read_text()
    assert 'RB-Y1' in interface and 'mock_move' not in interface  # no other robot's tools
    contract = json.loads((workspace/'MCP_CONTRACT.json').read_text())
    assert contract['robot'] == 'rby1' and set(ROBOT_FACTS['rby1']['tools']) <= set(contract['tools'])
    assert manifest['contract'] == contract
    agents = (workspace/'AGENTS.md').read_text()
    assert 'Rainbow RB-Y1' in agents and 'mock robot' not in agents


def test_subagent_text_only_with_subagents(tmp_path):
    build(recipe_for(), Setup('rby1', subagents=0), tmp_path/'none')
    build(recipe_for(), Setup('rby1', subagents=2), tmp_path/'two')
    assert 'subagents' not in (tmp_path/'none/AGENTS.md').read_text()
    assert "Claude Code's subagents" in (tmp_path/'two/AGENTS.md').read_text()


@pytest.mark.parametrize('robot', ['rby1', 'g1sonic'])
def test_method_skills_link_only_skills_the_robot_receives(tmp_path, robot):
    workspace = tmp_path/'ws'
    build(recipe_for(), Setup(robot, 'tidy_table'), workspace)
    skills = workspace/'.agents/skills'
    for name in (*METHOD_SKILLS, 'transfer-handoff'):
        text = (skills/name/'SKILL.md').read_text()
        assert '{{' not in text and '{%' not in text, name
        for link in re.findall(r'\]\((\.\./[^)]+)\)', text):
            assert (skills/name/link).resolve().is_file(), (name, link)
    tracking = ROBOT_FACTS[robot]['tracking_skill']
    assert f'../{tracking}/SKILL.md' in (skills/'engineer-the-controller/SKILL.md').read_text()


@pytest.mark.parametrize('count, kind, phrase', [
    (1, 'terminal_state', 'one image of the completed task'),
    (3, 'terminal_state', 'images of the completed task'),
    (4, 'completion_sequence', 'ordered image sequence'),
])
def test_in_context_action_learning_follows_the_demonstrations(tmp_path, count, kind, phrase):
    workspace = tmp_path/'ws'
    build(recipe_for(), Setup('g1sonic', 'tidy_table', demonstrations=count, demonstration_kind=kind), workspace)
    text = (workspace/'.agents/skills/in-context-action-learning/SKILL.md').read_text()
    assert phrase in text and 'runtime/demonstrations/' in text and '{%' not in text
    assert 'Unitree G1' in text
    agents = (workspace/'AGENTS.md').read_text()
    assert f'{count} demonstration image' in agents
    with pytest.raises(ValueError, match='demonstration kind'):
        Setup('g1sonic', demonstrations=1, demonstration_kind='video')


def test_blocks_registry_and_unknown_names_fail():
    assert set(registry()) == {'session.contract-pointer', 'images.forwarding', 'g1sonic.tracking-loop',
                             'g1sonic.field-guide-legend', 'g1sonic.perception-checks', 'g1sonic.robust-detection'}
    with pytest.raises(ValueError, match='Unknown workspace template name'):
        render('{{ nonexistent }}', Setup('rby1'))
    with pytest.raises(ValueError, match='Unknown workspace block'):
        render('{{ block("missing") }}', Setup('rby1'))


def test_every_template_variable_is_a_setup_fact():
    """Sources only use names Setup.context() and vocab provide (StrictUndefined renders all)."""
    for path in [*ROOT.glob('auto_research_agent/*.md'), *ROOT.glob('agent_skills/**/*.md'),
                 *ROOT.glob('agent_blocks/**/*.md')]:
        if path.name == 'README.md' and path.parent.name == 'agent_skills':
            continue
        for setup in (Setup('rby1'), Setup('g1sonic', demonstrations=2, demonstration_kind='completion_sequence')):
            render(path.read_text(), setup)


def test_agents_md_links_the_rby1_limits_and_posture_skills(tmp_path):
    workspace = tmp_path/'ws'
    build(recipe_for(), Setup('rby1', 'tidy_table', 'claude', 'rgbd'), workspace)
    text = (workspace/'AGENTS.md').read_text()
    links = set(re.findall(r'\]\((\.agents/skills/[^)]+)\)', text))
    assert all((workspace/link).is_file() for link in links)
    for skill in ('robot-interface', 'whole-body-control'):
        assert f'.agents/skills/{skill}/SKILL.md' in links
    for tool in ('rby1_fk', 'rby1_check'):
        assert tool in (workspace/'.agents/skills/robot-interface/SKILL.md').read_text()


@pytest.mark.parametrize('profile', ['rgbd', 'rgb-only'])
def test_a_team_workspace_keeps_each_robots_own_skills_apart(tmp_path, profile):
    workspace = tmp_path/'ws'
    build(recipe_for(), Setup('g1r1', 'carry_box', 'claude', profile, environment='sim'), workspace)
    shared, robots = workspace/'.agents/skills', workspace/'.agents/robots'
    assert {'robot-interface', 'multi-robot-task', 'object-localization', 'manual-robot-control', *METHOD_SKILLS} <= {
        p.name for p in shared.iterdir()}
    team = (shared/'robot-interface/SKILL.md').read_text()
    assert 'team_parallel' in team and 'semantic40' not in team             # the team's interface, not one robot's
    g1 = (robots/'g1/skills/robot-interface/SKILL.md').read_text()
    r1 = (robots/'r1/skills/robot-interface/SKILL.md').read_text()
    assert 'semantic40' in g1 and 'PiPER' in r1 and 'PiPER' not in g1
    assert '.agents/skills/' not in g1                                     # its paths name its own folder
    assert ('.agents/robots/g1/skills/robot-interface/scripts/g1_model.py' in g1) == (profile == 'rgbd')
    assert (robots/'g1/skills/controller-field-guide/SKILL.md').is_file()
    assert (robots/'r1/skills/object-localization').is_dir() == (profile == 'rgbd')
    for path in workspace.rglob('*.md'):
        text = path.read_text()
        assert '{{' not in text and '{%' not in text, path
        for link in re.findall(r'\]\(([^)#\s]+\.md)\)', text):
            if link.startswith('http') or '<' in link:
                continue
            target = workspace/link if link.startswith('.agents') else path.parent/link
            assert target.exists(), (path, link)                            # every link resolves, across the folders
    contract = json.loads((workspace/'MCP_CONTRACT.json').read_text())
    assert contract['robot'] == 'g1r1' and {'team_parallel', 'team_stop', 'g1_walk', 'r1_move_hand'} <= set(contract['tools'])
    agents = (workspace/'AGENTS.md').read_text()
    assert 'One arming, one stop' in agents and 'G1: **Arming, hold and stop.**' in agents and 'R1: **Arming and hold.**' in agents
    assert '[multi-robot-task](.agents/skills/multi-robot-task/SKILL.md)' in agents


def test_single_robot_workspaces_have_no_team_folders(tmp_path):
    build(recipe_for(), Setup('g1sonic', 'tidy_table'), tmp_path/'ws')
    assert not (tmp_path/'ws/.agents/robots').exists()
    assert not (tmp_path/'ws/.agents/skills/multi-robot-task').exists()


def test_skill_text_names_only_the_tools_the_session_offers(tmp_path):
    from services.workspace.contract_files import offered_tools
    options = json.loads((ROOT/'scripts/g1r1_sim_all_interfaces_options.json').read_text())
    setup = Setup('g1r1', 'carry_box', environment='sim')
    offered = offered_tools(setup, options)
    assert 'g1_crouch' not in offered and 'r1_grasp_paw' not in offered and {'g1_walk', 'r1_walk', 'r1_gait'} <= set(offered)
    build(recipe_for(), Setup('g1r1', 'carry_box', environment='sim', offered=offered), tmp_path/'hidden')
    build(recipe_for(), setup, tmp_path/'all')
    skill = '.agents/robots/g1/skills/robot-interface/SKILL.md'
    assert 'g1_crouch' not in (tmp_path/'hidden'/skill).read_text() and 'g1_walk' in (tmp_path/'hidden'/skill).read_text()
    assert 'g1_crouch' in (tmp_path/'all'/skill).read_text()
    for path in (tmp_path/'hidden').rglob('*.md'):
        if path.name not in ('MCP_SESSION.md',):
            assert not re.search(r'r1_(grasp_paw|place_in_box|leg_lift|carry_release|carry_hand)', path.read_text()), path
