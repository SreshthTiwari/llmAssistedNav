from setuptools import setup

package_name = 'final_proj'

setup(
    name=package_name,
    version='0.0.0',
    packages=[
        package_name,
        f'{package_name}.environment',
        f'{package_name}.planning',
        f'{package_name}.llm',
        f'{package_name}.memory',
        f'{package_name}.nodes',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='LLM-assisted uncertainty-aware navigation for autonomous delivery robots',
    license='MIT',
    entry_points={
        'console_scripts': [
            'nav_node = final_proj.nodes.nav_node:main',
        ],
    },
)