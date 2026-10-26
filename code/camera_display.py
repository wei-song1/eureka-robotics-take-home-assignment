"""Shared camera presentation for the Qt viewer and replay videos."""
from functools import lru_cache
import math

import numpy as np
from PIL import ImageDraw, ImageFont


def format_viewer_status(state, header=None):
    scene = state.get('scene') or {}
    scene_id = scene.get('id') or '--'
    name = str(scene.get('name') or 'Unknown').title()
    difficulty = str(scene.get('difficulty') or 'Unknown').title()
    target = state.get('target_goal')
    target_text = target if target not in (None, 'Manual') else 'None (Manual)'
    marked = target is not None and any(
        str(goal).casefold() == str(target).casefold()
        for goal in state.get('marked_goals', []))
    if header is None:
        time_text = '--'
        pose_text = 'x - --, y - --, -- deg'
    else:
        pose = header['pose']
        time_text = f"{header['t']:.2f}"
        pose_text = f"x - {pose['x']:.3f}, y - {pose['y']:.3f}, {math.degrees(pose['yaw']):.1f} deg"
    return (f'Scene {scene_id} - {name} ({difficulty})\n'
            f'Target Goal: {target_text}\n'
            f'Time: {time_text} s\n'
            f'Pose: {pose_text}\n'
            f"Goal: {'Marked' if marked else 'Unmarked'}")


def depth_preview(depth_u16, camera_info):
    """Fixed 0..max_depth grayscale: near is white, far/invalid is black."""
    # Keep invalid depth black instead of letting the scale turn it white.
    metres = depth_u16.astype(np.float32) * camera_info['depth_scale']
    brightness = np.clip(1 - metres / camera_info['max_depth'], 0, 1)
    image = (brightness * 255).astype(np.uint8)
    image[depth_u16 == 0] = 0
    return image


@lru_cache(maxsize=16)
def _font(size):
    for name in ('DejaVuSans.ttf', 'Arial.ttf', 'arial.ttf'):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()


def overlay_video_info(image, text):
    """White five-line status on a translucent box in the bottom-left corner."""
    result = image.convert('RGB').copy()
    width, height = result.size
    # Tiny test/special-purpose images cannot contain a readable status box.
    if width < 160 or height < 100:
        return result
    margin = max(4, width // 80)
    padding = max(4, width // 100)
    available = width - 2 * (margin + padding)
    font = _font(max(10, min(18, round(width * 18 / 640))))
    draw = ImageDraw.Draw(result, 'RGBA')
    lines = []
    for original in text.splitlines():
        line = original
        if draw.textlength(line, font=font) > available:
            while line and draw.textlength(line + '...', font=font) > available:
                line = line[:-1]
            line += '...'
        lines.append(line)
    text = '\n'.join(lines)
    spacing = 3
    left, top, right, bottom = draw.multiline_textbbox((0, 0), text, font=font, spacing=spacing)
    box_width = right - left + 2 * padding
    box_height = bottom - top + 2 * padding
    x, y = margin, height - margin - box_height
    draw.rectangle((x, y, x + box_width, y + box_height), fill=(0, 0, 0, 190))
    draw.multiline_text((x + padding - left, y + padding - top), text,
                        font=font, fill=(255, 255, 255, 255), spacing=spacing)
    return result
