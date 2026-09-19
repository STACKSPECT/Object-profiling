from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from .config import AppConfig, SCENE_PATH
from .contracts import Dimensions3D, snap_to_catalogue


@dataclass(frozen=True)
class BoxSpec:
    """Caja del episodio. Es generacion de escena, no contrato publico.

    Lleva las dimensiones reales, asi que no puede vivir en `contracts.py`: quien
    importe el contrato de integracion no debe llevarse el ground truth.
    """

    object_id: str
    dimensions_m: Dimensions3D
    mass_kg: float
    rgba: tuple[float, float, float, float]


# Detras de `scan_rgbd_cam` y sobre el suelo: ni entra en el encuadre ni
# penetra el plano del suelo mientras la estacion esta vacia.
BOX_PARKING_POSITION_M = np.asarray([2.5, 4.5, 0.5])


def _matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    quat = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, np.asarray(matrix, dtype=np.float64).reshape(9))
    return quat


def _set_free_joint_pose(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str, position: np.ndarray, rotation: np.ndarray) -> None:
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    address = model.jnt_qposadr[joint_id]
    data.qpos[address : address + 3] = position
    data.qpos[address + 3 : address + 7] = _matrix_to_quaternion(rotation)
    dof_address = model.jnt_dofadr[joint_id]
    data.qvel[dof_address : dof_address + 6] = 0.0


def _set_weld_from_current_pose(model: mujoco.MjModel, data: mujoco.MjData, equality_name: str) -> None:
    equality_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_EQUALITY, equality_name)
    body1_id = model.eq_obj1id[equality_id]
    body2_id = model.eq_obj2id[equality_id]
    rotation1 = data.xmat[body1_id].reshape(3, 3)
    rotation2 = data.xmat[body2_id].reshape(3, 3)
    relative_position = rotation1.T @ (data.xpos[body2_id] - data.xpos[body1_id])
    relative_rotation = rotation1.T @ rotation2
    model.eq_data[equality_id, 0:3] = 0.0
    model.eq_data[equality_id, 3:6] = relative_position
    model.eq_data[equality_id, 6:10] = _matrix_to_quaternion(relative_rotation)
    # Escala completa del par rotacional: este checkpoint modela la succion
    # como una union rigida ideal, no como una ventosa flexible.
    model.eq_data[equality_id, 10] = 1.0
    data.eq_active[equality_id] = 1


def _clamp_to_range(value: float, bounds: tuple[float, float]) -> float:
    return min(bounds[1], max(bounds[0], value))


def generate_box_spec(seed: int, config: AppConfig) -> BoxSpec:
    rng = np.random.default_rng(seed)
    length = float(rng.uniform(*config.box_range.length_m))
    width_upper = min(config.box_range.width_m[1], length)
    width = float(rng.uniform(config.box_range.width_m[0], width_upper))
    length, width = max(length, width), min(length, width)
    height = float(rng.uniform(*config.box_range.height_m))
    mass = float(rng.uniform(*config.box_range.mass_kg))
    color = tuple(float(value) for value in rng.uniform([0.45, 0.25, 0.08], [0.9, 0.65, 0.35])) + (1.0,)
    dimensions = Dimensions3D(length=length, width=width, height=height)
    catalogued = snap_to_catalogue(dimensions, config.catalogue_step_m)
    if catalogued is not None:
        dimensions = Dimensions3D(
            length=_clamp_to_range(catalogued.length, config.box_range.length_m),
            width=_clamp_to_range(catalogued.width, config.box_range.width_m),
            height=_clamp_to_range(catalogued.height, config.box_range.height_m),
        )
        length, width = max(dimensions.length, dimensions.width), min(dimensions.length, dimensions.width)
        dimensions = Dimensions3D(length=length, width=width, height=dimensions.height)
    return BoxSpec(
        object_id=f"box-{seed:04d}",
        dimensions_m=dimensions,
        mass_kg=mass,
        rgba=color,
    )


@dataclass
class ProfilingEnvironment:
    config: AppConfig
    box_spec: BoxSpec
    model: mujoco.MjModel
    data: mujoco.MjData
    box_visible: bool = True

    @classmethod
    def for_seed(
        cls,
        seed: int,
        config: AppConfig | None = None,
        *,
        attach_box: bool = True,
    ) -> "ProfilingEnvironment":
        """Construye el episodio de una seed. Es preparacion, no medicion."""

        config = config or AppConfig()
        return cls.create(generate_box_spec(seed, config), config, attach_box=attach_box)

    @property
    def object_id(self) -> str:
        """Etiqueta del objeto del episodio, sin revelar su geometria."""

        return self.box_spec.object_id

    @classmethod
    def create(
        cls,
        box_spec: BoxSpec,
        config: AppConfig | None = None,
        *,
        attach_box: bool = True,
    ) -> "ProfilingEnvironment":
        config = config or AppConfig()
        model = mujoco.MjModel.from_xml_path(str(SCENE_PATH))
        # Menagerie usa actuadores deliberadamente genericos. Estos ganhos
        # permiten sostener el terminal y una caja de hasta 5 kg sin modificar
        # el modelo cinematico del UR10e.
        model.actuator_gainprm[:, 0] = 15_000.0
        model.actuator_biasprm[:, 1] = -15_000.0
        model.actuator_biasprm[:, 2] = -1_200.0
        data = mujoco.MjData(model)
        environment = cls(config=config, box_spec=box_spec, model=model, data=data)
        environment.reset(attach_box=attach_box)
        return environment

    def load_box(self, box_spec: BoxSpec, *, attach_box: bool = False) -> None:
        """Reapunta el episodio a otra caja sin recargar el modelo.

        Las dimensiones de la caja viven en campos del modelo, asi que el mismo
        par modelo/datos sirve para varias cajas. Permite medir varias seguidas
        con un unico visor abierto, que es lo que MuJoCo admite por proceso.
        """

        self.box_spec = box_spec
        self.reset(attach_box=attach_box)

    def reset(self, *, attach_box: bool = True) -> None:
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[:6] = np.asarray(self.config.motion.home_qpos)
        self.data.ctrl[:] = np.asarray(self.config.motion.home_qpos)
        self._configure_box_model()
        mujoco.mj_forward(self.model, self.data)
        self._align_pickup_support()
        # Las geometrias estaticas se almacenan en constantes compiladas.
        # Recalcularlas antes de crear los welds hace efectiva la altura del
        # apoyo para cajas de cualquier altura del rango.
        mujoco.mj_setConst(self.model, self.data)
        # mj_setConst restaura temporalmente qpos0; recuperar la pose inicial
        # declarada antes de colocar el terminal y la caja.
        self.data.qpos[:6] = np.asarray(self.config.motion.home_qpos)
        self.data.ctrl[:] = np.asarray(self.config.motion.home_qpos)
        mujoco.mj_forward(self.model, self.data)
        self._place_and_attach_gripper()
        self._place_box()
        if attach_box:
            self.attach_box()
        mujoco.mj_forward(self.model, self.data)

    def _configure_box_model(self) -> None:
        dimensions = self.box_spec.dimensions_m
        geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "profiling_box")
        self.model.geom_size[geom_id] = dimensions.as_array() / 2.0
        self.model.geom_rgba[geom_id] = self.box_spec.rgba
        self.model.body_mass[body_id] = self.box_spec.mass_kg
        # Deshace un posible aparcamiento previo de la caja.
        self.model.geom_contype[geom_id] = 1
        self.model.geom_conaffinity[geom_id] = 1
        self.box_visible = True
        length, width, height = dimensions.as_array()
        self.model.body_inertia[body_id] = self.box_spec.mass_kg / 12.0 * np.asarray(
            [width * width + height * height, length * length + height * height, length * length + width * width]
        )

    def _place_and_attach_gripper(self) -> None:
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
        rotation = self.data.site_xmat[site_id].reshape(3, 3).copy()
        position = self.data.site_xpos[site_id].copy()
        _set_free_joint_pose(self.model, self.data, "gripper_free", position, rotation)
        mujoco.mj_forward(self.model, self.data)
        _set_weld_from_current_pose(self.model, self.data, "wrist_to_gripper")

    def _place_box(self) -> None:
        gripper_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "profiling_gripper")
        rotation = self.data.xmat[gripper_id].reshape(3, 3).copy()
        offset = self.config.sensor.tool_to_box_offset_m + self.box_spec.dimensions_m.height / 2.0
        position = self.data.xpos[gripper_id] + rotation @ np.asarray([0.0, 0.0, offset])
        _set_free_joint_pose(self.model, self.data, "box_free", position, rotation)
        mujoco.mj_forward(self.model, self.data)

    def _align_pickup_support(self) -> None:
        """Coloca el apoyo a una altura fija, independiente de la caja.

        Antes se alineaba con la base de la caja concreta, usando su altura real.
        Eso hacia que el fondo de calibracion dependiese de la caja que se iba a
        medir: no se podia reutilizar, y el procedimiento era circular, porque
        exigia conocer una altura que todavia no se ha medido. Ahora se alinea
        con la caja mas alta del rango declarado, que es informacion de diseno.
        """

        attachment_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
        support_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "pickup_support")
        support_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "pickup_support_geom")
        lowest_box_bottom_z = (
            self.data.site_xpos[attachment_site_id, 2]
            - self.config.sensor.tool_to_box_offset_m
            - self.config.box_range.height_m[1]
        )
        support_half_height = self.model.geom_size[support_geom_id, 2]
        self.model.geom_pos[support_geom_id, 2] = (
            lowest_box_bottom_z - self.data.xpos[support_body_id, 2] - support_half_height
        )
        mujoco.mj_forward(self.model, self.data)

    def attach_box(self) -> None:
        """Activa la abstraccion declarada de succion en la pose actual."""

        _set_weld_from_current_pose(self.model, self.data, "gripper_to_box")
        mujoco.mj_forward(self.model, self.data)

    def detach_box(self) -> None:
        equality_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "gripper_to_box")
        self.data.eq_active[equality_id] = 0
        mujoco.mj_forward(self.model, self.data)

    @property
    def box_attached(self) -> bool:
        equality_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "gripper_to_box")
        return bool(self.data.eq_active[equality_id])

    def set_box_visible(self, visible: bool) -> None:
        """Retira o restituye la caja como preparacion de escena.

        Ocultarla deja la estacion como estaria sin paquete: detras de
        `scan_rgbd_cam` y sin contactos. Aparcarla bajo el suelo la hacia
        penetrar el plano y salir despedida hasta alturas todavia dentro del
        plano lejano del render.

        Llamarla con `False` de nuevo vuelve a aparcarla. La caja aparcada cae
        libremente porque `body_gravcomp` no actua sobre un freejoint, asi que
        reaparcarla antes de cada captura de fondo la mantiene en un sitio
        conocido.
        """

        box_geom_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
        equality_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "gripper_to_box")
        if visible:
            self.model.geom_contype[box_geom_id] = 1
            self.model.geom_conaffinity[box_geom_id] = 1
            self._place_box()
            self.attach_box()
        else:
            self.data.eq_active[equality_id] = 0
            self.model.geom_contype[box_geom_id] = 0
            self.model.geom_conaffinity[box_geom_id] = 0
            _set_free_joint_pose(self.model, self.data, "box_free", BOX_PARKING_POSITION_M, np.eye(3))
        self.box_visible = visible
        mujoco.mj_forward(self.model, self.data)

    def set_joint_positions(self, joint_positions_rad: np.ndarray) -> None:
        """Coloca el brazo en una configuracion articular medida.

        El terminal es un cuerpo libre unido por weld, y un weld solo se
        resuelve al integrar. Tras teletransportar el brazo hay que recolocarlo
        explicitamente para que la escena sea coherente sin simular.
        """

        self.data.qpos[:6] = np.asarray(joint_positions_rad, dtype=np.float64)
        self.data.qvel[:6] = 0.0
        self.data.ctrl[:] = np.asarray(joint_positions_rad, dtype=np.float64)
        mujoco.mj_forward(self.model, self.data)
        self._place_and_attach_gripper()
        mujoco.mj_forward(self.model, self.data)

    def body_to_world(self, body_name: str) -> np.ndarray:
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        transform = np.eye(4)
        transform[:3, :3] = self.data.xmat[body_id].reshape(3, 3)
        transform[:3, 3] = self.data.xpos[body_id]
        return transform

    def gripper_to_box(self) -> np.ndarray:
        gripper_to_world = self.body_to_world("profiling_gripper")
        box_to_world = self.body_to_world("profiling_box")
        return np.linalg.inv(gripper_to_world) @ box_to_world

    def tool_to_world(self) -> np.ndarray:
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "attachment_site")
        transform = np.eye(4)
        transform[:3, :3] = self.data.site_xmat[site_id].reshape(3, 3)
        transform[:3, 3] = self.data.site_xpos[site_id]
        return transform

    def active_cup_names(self) -> tuple[str, ...]:
        half_length = self.box_spec.dimensions_m.length / 2.0
        half_width = self.box_spec.dimensions_m.width / 2.0
        cups = {
            "cup_center": (0.0, 0.0),
            "cup_x_pos": (0.055, 0.0),
            "cup_x_neg": (-0.055, 0.0),
            "cup_y_pos": (0.0, 0.040),
            "cup_y_neg": (0.0, -0.040),
        }
        radius = 0.020
        return tuple(
            name
            for name, (x, y) in cups.items()
            if abs(x) + radius <= half_length and abs(y) + radius <= half_width
        )
