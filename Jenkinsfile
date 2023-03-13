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