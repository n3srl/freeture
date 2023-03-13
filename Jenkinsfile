pipeline {
    agent  any
    stages {
        stage('Build') 
		{
			agent  {
			   dockerfile {
					filename '/home/nova/Repository/PRISMA_NODE_DOCKER/freeture-dev/Dockerfile'
					dir 'build'
					label 'freeture-dev'
					#additionalBuildArgs  '--build-arg version=1.0.2'
					#args '-v /tmp:/tmp'
				}
			}
            steps {
                echo 'Building..'
            }
        }
        stage('Test') 
		{
            steps {
                echo 'Testing..'
            }
        }
    }
}